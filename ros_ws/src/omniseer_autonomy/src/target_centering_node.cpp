#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_autonomy/target_centering_controller.hpp"
#include "omniseer_msgs/srv/capture_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace omniseer_autonomy
{
  namespace
  {
    using namespace std::chrono_literals;

    std::string json_escape(const std::string& value)
    {
      std::string out;
      out.reserve(value.size() + 8);
      for (const char ch : value)
      {
        switch (ch)
        {
        case '\\':
          out += "\\\\";
          break;
        case '"':
          out += "\\\"";
          break;
        case '\n':
          out += "\\n";
          break;
        case '\r':
          out += "\\r";
          break;
        case '\t':
          out += "\\t";
          break;
        default:
          out += ch;
          break;
        }
      }
      return out;
    }

    std::string optional_json_number(std::optional<double> value)
    {
      if (!value.has_value())
      {
        return "null";
      }
      std::ostringstream stream;
      stream << *value;
      return stream.str();
    }

    std::string optional_log_number(std::optional<double> value)
    {
      if (!value.has_value())
      {
        return "-";
      }
      std::ostringstream stream;
      stream << *value;
      return stream.str();
    }

    double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& orientation)
    {
      const auto siny_cosp =
          2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y));
      const auto cosy_cosp =
          1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)));
      return std::atan2(siny_cosp, cosy_cosp);
    }
  } // namespace

  class TargetCenteringNode : public rclcpp::Node
  {
  public:
    explicit TargetCenteringNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("target_centering_node", options),
          _started_at(std::chrono::steady_clock::now()), _controller(make_config())
    {
      const auto run_dir = declare_parameter<std::string>("run_dir", "");
      _frame_id          = declare_parameter<std::string>("cmd_frame_id", "base_link");

      if (!run_dir.empty())
      {
        const auto      path = std::filesystem::path(run_dir) / "autonomy.jsonl";
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        if (!ec)
        {
          _events.open(path, std::ios::out | std::ios::trunc);
        }
        if (!_events.is_open())
        {
          RCLCPP_WARN(get_logger(), "failed to open autonomy event log: %s", path.c_str());
        }
      }

      const auto detections_topic =
          declare_parameter<std::string>("detections_topic", "/yolo/detections");
      const auto odometry_topic =
          declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
      const auto range_topic = declare_parameter<std::string>("range_topic", "/range");
      const auto command_topic =
          declare_parameter<std::string>("command_topic", "/cmd_vel_autonomy");
      const auto capture_service =
          declare_parameter<std::string>("capture_service", "/vision/capture_frame");
      _capture_timeout_sec  = declare_parameter<double>("capture_timeout_sec", 2.0);
      _shutdown_on_terminal = declare_parameter<bool>("shutdown_on_terminal", true);
      _publisher            = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic, 10);
      _capture_client       = create_client<omniseer_msgs::srv::CaptureFrame>(capture_service);
      _subscription         = create_subscription<yolo_msgs::msg::DetectionArray>(
          detections_topic, 10,
          [this](const yolo_msgs::msg::DetectionArray& msg)
          {
            std::vector<TargetDetection> detections;
            detections.reserve(msg.detections.size());
            for (const auto& detection : msg.detections)
            {
              detections.push_back(TargetDetection{
                  detection.class_name,
                  static_cast<double>(detection.score),
                  static_cast<double>(detection.bbox.center.position.x),
                  static_cast<double>(detection.bbox.center.position.y),
                  static_cast<double>(detection.bbox.size.x),
                  static_cast<double>(detection.bbox.size.y),
              });
            }
            apply_output(_controller.update_detections(detections, elapsed_sec()));
          });
      _odometry_subscription = create_subscription<nav_msgs::msg::Odometry>(
          odometry_topic, 10, [this](const nav_msgs::msg::Odometry& msg)
          { _controller.update_heading(yaw_from_quaternion(msg.pose.pose.orientation)); });
      _range_subscription = create_subscription<sensor_msgs::msg::Range>(
          range_topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::Range& msg)
          {
            if (std::isfinite(msg.range))
            {
              _controller.update_proximity_range(static_cast<double>(msg.range));
            }
            else
            {
              _controller.update_proximity_range(std::nullopt);
            }
          });
      _timer = create_wall_timer(50ms, [this]() { apply_output(_controller.tick(elapsed_sec())); });

      RCLCPP_INFO(
          get_logger(),
          "target centering autonomy started; target_class=%s command_topic=%s detections_topic=%s "
          "odometry_topic=%s range_topic=%s capture_service=%s",
          _target_class.c_str(), command_topic.c_str(), detections_topic.c_str(),
          odometry_topic.c_str(), range_topic.c_str(), capture_service.c_str());
    }

    ~TargetCenteringNode() override
    {
      publish_command(0.0, 0.0);
      if (_events.is_open())
      {
        _events.flush();
        _events.close();
      }
    }

  private:
    TargetCenteringConfig make_config()
    {
      TargetCenteringConfig config{};
      config.target_class            = declare_parameter<std::string>("target_class", "");
      config.image_width_px          = declare_parameter<double>("image_width_px", 1280.0);
      config.image_height_px         = declare_parameter<double>("image_height_px", 720.0);
      config.scan_yaw_rate_rad_s     = declare_parameter<double>("scan_yaw_rate_rad_s", 0.20);
      config.max_yaw_rate_rad_s      = declare_parameter<double>("max_yaw_rate_rad_s", 0.30);
      config.min_yaw_rate_rad_s      = declare_parameter<double>("min_yaw_rate_rad_s", 0.08);
      config.kp                      = declare_parameter<double>("kp", 0.30);
      config.center_deadband         = declare_parameter<double>("center_deadband", 0.05);
      config.bbox_area_min_ratio     = declare_parameter<double>("bbox_area_min_ratio", 0.08);
      config.bbox_area_max_ratio     = declare_parameter<double>("bbox_area_max_ratio", 0.35);
      config.forward_speed_m_s       = declare_parameter<double>("forward_speed_m_s", 0.05);
      config.reverse_speed_m_s       = declare_parameter<double>("reverse_speed_m_s", 0.04);
      config.stable_framed_frames    = declare_parameter<int>("stable_framed_frames", 10);
      config.proximity_stop_m        = declare_parameter<double>("proximity_stop_m", 0.30);
      const auto detection_stale_ms  = declare_parameter<int>("detection_stale_ms", 500);
      config.detection_stale_sec     = static_cast<double>(detection_stale_ms) / 1000.0;
      config.scan_limit_revolutions  = declare_parameter<double>("scan_limit_revolutions", 1.0);
      config.target_lost_timeout_sec = declare_parameter<double>("target_lost_timeout_sec", 0.5);
      _target_class                  = config.target_class;
      return config;
    }

    double elapsed_sec() const
    {
      const auto elapsed = std::chrono::steady_clock::now() - _started_at;
      return std::chrono::duration<double>(elapsed).count();
    }

    void apply_output(const TargetCenteringOutput& output)
    {
      for (const auto& event : output.events)
      {
        write_event(event);
        log_state_transition(event);
      }
      if (output.publish_command)
      {
        publish_command(output.linear_x_m_s, output.angular_z_rad_s);
      }
      if (!_terminal_logged && _controller.state() == CenteringState::Success)
      {
        _terminal_logged = true;
        RCLCPP_INFO(get_logger(), "target centering succeeded");
        log_terminal_summary();
        if (!request_capture(output))
        {
          request_terminal_shutdown("target centering succeeded");
        }
      }
      else if (!_terminal_logged && _controller.state() == CenteringState::Failed)
      {
        _terminal_logged = true;
        RCLCPP_WARN(get_logger(), "target centering failed: %s",
                    _controller.terminal_reason().c_str());
        log_terminal_summary();
        request_terminal_shutdown("target centering failed");
      }
    }

    void publish_command(double linear_x_m_s, double angular_z_rad_s)
    {
      if (!_publisher)
      {
        return;
      }
      geometry_msgs::msg::TwistStamped msg{};
      msg.header.stamp    = now();
      msg.header.frame_id = _frame_id;
      msg.twist.linear.x  = linear_x_m_s;
      msg.twist.angular.z = angular_z_rad_s;
      _publisher->publish(msg);
    }

    void write_event(const TargetCenteringEvent& event)
    {
      if (!_events.is_open())
      {
        return;
      }
      _events << "{\"schema_version\":1"
              << ",\"time_sec\":" << event.time_sec << ",\"state\":\"" << state_name(event.state)
              << "\""
              << ",\"event\":\"" << json_escape(event.event) << "\""
              << ",\"reason\":\"" << json_escape(event.reason) << "\""
              << ",\"target_class\":\"" << json_escape(_target_class) << "\""
              << ",\"normalized_error\":" << optional_json_number(event.normalized_error)
              << ",\"bbox_area_ratio\":" << optional_json_number(event.bbox_area_ratio)
              << ",\"proximity_range_m\":" << optional_json_number(event.proximity_range_m)
              << ",\"linear_x_m_s\":" << event.linear_x_m_s
              << ",\"angular_z_rad_s\":" << event.angular_z_rad_s
              << ",\"stable_framed_frames\":" << event.stable_framed_frames
              << ",\"target_loss_count\":" << event.target_loss_count;
      if (event.target.has_value())
      {
        _events << ",\"target\":{\"class_name\":\"" << json_escape(event.target->class_name)
                << "\",\"confidence\":" << event.target->confidence
                << ",\"bbox\":{\"center_x\":" << event.target->center_x_px
                << ",\"center_y\":" << event.target->center_y_px
                << ",\"size_x\":" << event.target->size_x_px
                << ",\"size_y\":" << event.target->size_y_px << "}}";
      }
      else
      {
        _events << ",\"target\":null";
      }
      _events << "}\n";
      _events.flush();
    }

    void log_state_transition(const TargetCenteringEvent& event)
    {
      remember_state(event.state);
      if (_last_logged_state.has_value() && *_last_logged_state == event.state)
      {
        return;
      }

      std::ostringstream message;
      message << "autonomy state reached: " << state_name(event.state) << " event=" << event.event;
      if (_last_logged_state.has_value())
      {
        message << " previous_state=" << state_name(*_last_logged_state);
      }
      if (!event.reason.empty())
      {
        message << " reason=" << event.reason;
      }
      if (event.target.has_value())
      {
        message << " target=" << event.target->class_name
                << " confidence=" << event.target->confidence;
      }
      message << " normalized_error=" << optional_log_number(event.normalized_error)
              << " bbox_area_ratio=" << optional_log_number(event.bbox_area_ratio)
              << " stable_framed_frames=" << event.stable_framed_frames
              << " target_loss_count=" << event.target_loss_count;

      const auto text = message.str();
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
      _last_logged_state = event.state;
    }

    void remember_state(CenteringState state)
    {
      for (const auto reached : _states_reached)
      {
        if (reached == state)
        {
          return;
        }
      }
      _states_reached.push_back(state);
    }

    std::string states_reached_text() const
    {
      std::ostringstream stream;
      for (std::size_t i = 0; i < _states_reached.size(); ++i)
      {
        if (i > 0)
        {
          stream << ",";
        }
        stream << state_name(_states_reached[i]);
      }
      return stream.str();
    }

    void log_terminal_summary()
    {
      remember_state(_controller.state());
      const auto         reason = _controller.terminal_reason();
      std::ostringstream message;
      message << "autonomy summary: terminal_state=" << state_name(_controller.state())
              << " reason=" << (reason.empty() ? "-" : reason)
              << " states_reached=" << states_reached_text()
              << " target_loss_count=" << _controller.target_loss_count()
              << " time_to_first_detection_sec="
              << optional_log_number(_controller.time_to_first_detection_sec())
              << " time_to_centered_sec=" << optional_log_number(_controller.time_to_centered_sec())
              << " final_error=" << optional_log_number(_controller.final_error())
              << " final_confidence=" << optional_log_number(_controller.final_confidence());
      const auto text = message.str();
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }

    bool request_capture(const TargetCenteringOutput& output)
    {
      if (_capture_requested)
      {
        return false;
      }
      _capture_requested = true;
      if (!_capture_client)
      {
        write_capture_result(false, "capture_client_unavailable", "", 0, 0);
        return false;
      }
      if (!_capture_client->service_is_ready() &&
          !_capture_client->wait_for_service(std::chrono::milliseconds(100)))
      {
        write_capture_result(false, "capture_service_unavailable", "", 0, 0);
        return false;
      }

      auto request = make_capture_request(output);
      if (!request.has_value())
      {
        write_capture_result(false, "capture_target_unavailable", "", 0, 0);
        return false;
      }

      using ServiceResponseFuture = rclcpp::Client<omniseer_msgs::srv::CaptureFrame>::SharedFuture;
      arm_capture_timeout();
      _capture_client->async_send_request(
          std::make_shared<omniseer_msgs::srv::CaptureFrame::Request>(std::move(*request)),
          [this](ServiceResponseFuture future)
          {
            if (_capture_completed)
            {
              return;
            }
            _capture_completed = true;
            if (_capture_timeout_timer)
            {
              _capture_timeout_timer->cancel();
            }
            const auto response = future.get();
            write_capture_result(response->success, response->reason, response->image_path,
                                 response->frame_id, response->sequence);
            if (response->success)
            {
              RCLCPP_INFO(get_logger(), "target capture saved: %s", response->image_path.c_str());
            }
            else
            {
              RCLCPP_WARN(get_logger(), "target capture failed: %s", response->reason.c_str());
            }
            request_terminal_shutdown("target capture completed");
          });
      return true;
    }

    void arm_capture_timeout()
    {
      const auto timeout_sec = _capture_timeout_sec > 0.0 ? _capture_timeout_sec + 1.0 : 1.0;
      const auto timeout     = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(timeout_sec));
      _capture_timeout_timer =
          create_wall_timer(timeout,
                            [this]()
                            {
                              if (_capture_completed)
                              {
                                return;
                              }
                              _capture_completed = true;
                              if (_capture_timeout_timer)
                              {
                                _capture_timeout_timer->cancel();
                              }
                              write_capture_result(false, "capture_response_timeout", "", 0, 0);
                              RCLCPP_WARN(get_logger(), "target capture timed out");
                              request_terminal_shutdown("target capture timed out");
                            });
    }

    std::optional<omniseer_msgs::srv::CaptureFrame::Request> make_capture_request(
        const TargetCenteringOutput& output) const
    {
      for (auto it = output.events.rbegin(); it != output.events.rend(); ++it)
      {
        if (!it->target.has_value() || !it->normalized_error.has_value() ||
            !it->bbox_area_ratio.has_value())
        {
          continue;
        }

        omniseer_msgs::srv::CaptureFrame::Request request{};
        request.capture_reason   = "target_framed";
        request.target_class     = _target_class;
        request.confidence       = it->target->confidence;
        request.bbox_center_x_px = it->target->center_x_px;
        request.bbox_center_y_px = it->target->center_y_px;
        request.bbox_size_x_px   = it->target->size_x_px;
        request.bbox_size_y_px   = it->target->size_y_px;
        request.normalized_error = *it->normalized_error;
        request.bbox_area_ratio  = *it->bbox_area_ratio;
        request.timeout_sec      = _capture_timeout_sec;
        return request;
      }
      return std::nullopt;
    }

    void write_capture_result(bool success, const std::string& reason,
                              const std::string& image_path, uint64_t frame_id, uint64_t sequence)
    {
      if (!_events.is_open())
      {
        return;
      }
      _events << "{\"schema_version\":1"
              << ",\"time_sec\":" << elapsed_sec() << ",\"state\":\""
              << state_name(_controller.state()) << "\""
              << ",\"event\":\"capture_result\""
              << ",\"reason\":\"" << json_escape(reason) << "\""
              << ",\"target_class\":\"" << json_escape(_target_class) << "\""
              << ",\"success\":" << (success ? "true" : "false") << ",\"image_path\":\""
              << json_escape(image_path) << "\""
              << ",\"frame_id\":" << frame_id << ",\"sequence\":" << sequence << "}\n";
      _events.flush();
    }

    void request_terminal_shutdown(const std::string& reason)
    {
      if (!_shutdown_on_terminal || _terminal_shutdown_requested)
      {
        return;
      }
      _terminal_shutdown_requested = true;
      if (_timer)
      {
        _timer->cancel();
      }
      if (_capture_timeout_timer)
      {
        _capture_timeout_timer->cancel();
      }
      RCLCPP_INFO(get_logger(), "terminal autonomy handling complete; shutting down: %s",
                  reason.c_str());
      rclcpp::shutdown();
    }

    std::chrono::steady_clock::time_point                           _started_at{};
    std::string                                                     _target_class{};
    std::string                                                     _frame_id{"base_link"};
    double                                                          _capture_timeout_sec{2.0};
    bool                                                            _shutdown_on_terminal{true};
    TargetCenteringController                                       _controller;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  _publisher{};
    rclcpp::Client<omniseer_msgs::srv::CaptureFrame>::SharedPtr     _capture_client{};
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr _subscription{};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        _odometry_subscription{};
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr        _range_subscription{};
    rclcpp::TimerBase::SharedPtr                                    _timer{};
    rclcpp::TimerBase::SharedPtr                                    _capture_timeout_timer{};
    std::ofstream                                                   _events{};
    std::optional<CenteringState>                                   _last_logged_state{};
    std::vector<CenteringState>                                     _states_reached{};
    bool                                                            _terminal_logged{false};
    bool                                                            _capture_requested{false};
    bool                                                            _capture_completed{false};
    bool _terminal_shutdown_requested{false};
  };
} // namespace omniseer_autonomy

#ifndef OMNISEER_AUTONOMY_TESTING
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<omniseer_autonomy::TargetCenteringNode>());
  }
  catch (const std::exception& exc)
  {
    RCLCPP_FATAL(rclcpp::get_logger("target_centering_node"), "target centering node failed: %s",
                 exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
