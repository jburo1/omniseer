#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "omniseer_autonomy/target_centering_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace omniseer_autonomy
{
namespace
{
using namespace std::chrono_literals;

std::string json_escape(const std::string & value)
{
  std::string out;
  out.reserve(value.size() + 8);
  for (const char ch : value) {
    switch (ch) {
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
  if (!value.has_value()) {
    return "null";
  }
  std::ostringstream stream;
  stream << *value;
  return stream.str();
}
} // namespace

class TargetCenteringNode : public rclcpp::Node
{
public:
  TargetCenteringNode()
  : rclcpp::Node("target_centering_node"),
    _started_at(std::chrono::steady_clock::now()),
    _controller(make_config())
  {
    const auto run_dir = declare_parameter<std::string>("run_dir", "");
    _frame_id = declare_parameter<std::string>("cmd_frame_id", "base_link");

    if (!run_dir.empty()) {
      const auto path = std::filesystem::path(run_dir) / "autonomy.jsonl";
      std::error_code ec;
      std::filesystem::create_directories(path.parent_path(), ec);
      if (!ec) {
        _events.open(path, std::ios::out | std::ios::trunc);
      }
      if (!_events.is_open()) {
        RCLCPP_WARN(get_logger(), "failed to open autonomy event log: %s", path.c_str());
      }
    }

    const auto detections_topic = declare_parameter<std::string>("detections_topic",
        "/yolo/detections");
    const auto command_topic = declare_parameter<std::string>("command_topic", "/cmd_vel_autonomy");
    _publisher = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic, 10);
    _subscription = create_subscription<yolo_msgs::msg::DetectionArray>(
      detections_topic, 10,
      [this](const yolo_msgs::msg::DetectionArray & msg)
      {
        std::vector<TargetDetection> detections;
        detections.reserve(msg.detections.size());
        for (const auto & detection : msg.detections) {
          detections.push_back(
            TargetDetection{
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
    _timer = create_wall_timer(
      50ms,
      [this]()
      {
        apply_output(_controller.tick(elapsed_sec()));
      });

    RCLCPP_INFO(
      get_logger(),
        "target centering autonomy started; target_class=%s command_topic=%s detections_topic=%s",
      _target_class.c_str(), command_topic.c_str(), detections_topic.c_str());
  }

  ~TargetCenteringNode() override
  {
    publish_command(0.0);
    if (_events.is_open()) {
      _events.flush();
      _events.close();
    }
  }

private:
  TargetCenteringConfig make_config()
  {
    TargetCenteringConfig config{};
    config.target_class = declare_parameter<std::string>("target_class", "");
    config.image_width_px = declare_parameter<double>("image_width_px", 1280.0);
    config.scan_yaw_rate_rad_s = declare_parameter<double>("scan_yaw_rate_rad_s", 0.20);
    config.max_yaw_rate_rad_s = declare_parameter<double>("max_yaw_rate_rad_s", 0.30);
    config.min_yaw_rate_rad_s = declare_parameter<double>("min_yaw_rate_rad_s", 0.08);
    config.kp = declare_parameter<double>("kp", 0.30);
    config.center_deadband = declare_parameter<double>("center_deadband", 0.05);
    config.stable_center_frames = declare_parameter<int>("stable_center_frames", 10);
    const auto detection_stale_ms = declare_parameter<int>("detection_stale_ms", 500);
    config.detection_stale_sec = static_cast<double>(detection_stale_ms) / 1000.0;
    config.scan_timeout_sec = declare_parameter<double>("scan_timeout_sec", 12.0);
    config.target_lost_timeout_sec = declare_parameter<double>("target_lost_timeout_sec", 0.5);
    _target_class = config.target_class;
    return config;
  }

  double elapsed_sec() const
  {
    const auto elapsed = std::chrono::steady_clock::now() - _started_at;
    return std::chrono::duration<double>(elapsed).count();
  }

  void apply_output(const TargetCenteringOutput & output)
  {
    for (const auto & event : output.events) {
      write_event(event);
    }
    if (output.publish_command) {
      publish_command(output.angular_z_rad_s);
    }
    if (!_terminal_logged && _controller.state() == CenteringState::Success) {
      _terminal_logged = true;
      RCLCPP_INFO(get_logger(), "target centering succeeded");
    } else if (!_terminal_logged && _controller.state() == CenteringState::Failed) {
      _terminal_logged = true;
      RCLCPP_WARN(get_logger(), "target centering failed: %s",
          _controller.terminal_reason().c_str());
    }
  }

  void publish_command(double angular_z_rad_s)
  {
    if (!_publisher) {
      return;
    }
    geometry_msgs::msg::TwistStamped msg{};
    msg.header.stamp = now();
    msg.header.frame_id = _frame_id;
    msg.twist.angular.z = angular_z_rad_s;
    _publisher->publish(msg);
  }

  void write_event(const TargetCenteringEvent & event)
  {
    if (!_events.is_open()) {
      return;
    }
    _events << "{\"schema_version\":1"
            << ",\"time_sec\":" << event.time_sec
            << ",\"state\":\"" << state_name(event.state) << "\""
            << ",\"event\":\"" << json_escape(event.event) << "\""
            << ",\"reason\":\"" << json_escape(event.reason) << "\""
            << ",\"target_class\":\"" << json_escape(_target_class) << "\""
            << ",\"normalized_error\":" << optional_json_number(event.normalized_error)
            << ",\"angular_z_rad_s\":" << event.angular_z_rad_s
            << ",\"stable_center_frames\":" << event.stable_center_frames
            << ",\"target_loss_count\":" << event.target_loss_count;
    if (event.target.has_value()) {
      _events << ",\"target\":{\"class_name\":\"" << json_escape(event.target->class_name)
              << "\",\"confidence\":" << event.target->confidence
              << ",\"bbox\":{\"center_x\":" << event.target->center_x_px
              << ",\"center_y\":" << event.target->center_y_px
              << ",\"size_x\":" << event.target->size_x_px
              << ",\"size_y\":" << event.target->size_y_px << "}}";
    } else {
      _events << ",\"target\":null";
    }
    _events << "}\n";
    _events.flush();
  }

  std::chrono::steady_clock::time_point _started_at{};
  std::string                           _target_class{};
  std::string                           _frame_id{"base_link"};
  TargetCenteringController             _controller;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher{};
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr _subscription{};
  rclcpp::TimerBase::SharedPtr _timer{};
  std::ofstream                _events{};
  bool                         _terminal_logged{false};
};
} // namespace omniseer_autonomy

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<omniseer_autonomy::TargetCenteringNode>());
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("target_centering_node"), "target centering node failed: %s",
      exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
