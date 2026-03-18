#include <array>
#include <chrono>
#include <exception>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer_vision_bridge/vision_bridge_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace omniseer_vision_bridge
{
namespace
{
double ns_to_ms(uint64_t ns) noexcept
{
  return static_cast<double>(ns) / 1.0e6;
}

builtin_interfaces::msg::Time to_builtin_time(uint64_t real_ns) noexcept
{
  builtin_interfaces::msg::Time out{};
  out.sec = static_cast<int32_t>(real_ns / 1000000000ULL);
  out.nanosec = static_cast<uint32_t>(real_ns % 1000000000ULL);
  return out;
}
}   // namespace

class RosYoloDetectionsSink final : public omniseer::vision::IDetectionsSink
{
public:
  RosYoloDetectionsSink(
    rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr publisher,
    std::vector<std::string> class_names, std::string camera_frame_id)
  : _publisher(std::move(publisher)), _class_names(std::move(class_names)),
    _camera_frame_id(std::move(camera_frame_id))
  {
  }

  void publish(const omniseer::vision::DetectionsFrame & frame) noexcept override
  {
    if (!_publisher) {
      return;
    }

    yolo_msgs::msg::DetectionArray msg{};
    msg.header.stamp = to_builtin_time(frame.capture_ts_real_ns);
    msg.header.frame_id = _camera_frame_id;
    msg.detections.reserve(frame.count);

    for (uint32_t i = 0; i < frame.count; ++i) {
      const auto & det = frame.detections[i];
      yolo_msgs::msg::Detection out{};
      out.class_id = static_cast<int32_t>(det.class_id);
      if (det.class_id < _class_names.size()) {
        out.class_name = _class_names[det.class_id];
      }
      out.score = static_cast<double>(det.score);
      out.bbox.center.position.x = static_cast<double>((det.x1 + det.x2) * 0.5F);
      out.bbox.center.position.y = static_cast<double>((det.y1 + det.y2) * 0.5F);
      out.bbox.center.theta = 0.0;
      out.bbox.size.x = static_cast<double>(det.x2 - det.x1);
      out.bbox.size.y = static_cast<double>(det.y2 - det.y1);
      msg.detections.emplace_back(std::move(out));
    }

    _publisher->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr _publisher{};
  std::vector<std::string> _class_names{};
  std::string _camera_frame_id{};
};

class VisionBridgeNode final : public rclcpp::Node
{
public:
  VisionBridgeNode()
  : rclcpp::Node("vision_bridge")
  {
    _config = declare_bridge_parameters();
    log_config(_config);
    _detections_publisher =
      create_publisher<yolo_msgs::msg::DetectionArray>("/yolo/detections", 10);
    _detections_sink = std::make_unique<RosYoloDetectionsSink>(
          _detections_publisher,
          omniseer::vision::load_class_list_file(_config.class_list_path),
          _config.camera_frame_id);
    _perf_publisher =
      create_publisher<omniseer_msgs::msg::VisionPerfSummary>("/vision/perf", 1);
    _config.detections_sink = _detections_sink.get();
    _runtime = std::make_unique<VisionBridgeRuntime>(_config);
    _runtime->start();
    RCLCPP_INFO(get_logger(),
                  "vision runtime started in headless bridge mode and publishing /yolo/detections and /vision/perf");

    _last_perf_time = std::chrono::steady_clock::now();
    _last_perf_snapshot = _runtime->telemetry_snapshot();
    _perf_timer = create_wall_timer(std::chrono::milliseconds(500), [this]()
        {
          publish_perf_summary();
      });

    _health_timer = create_wall_timer(std::chrono::milliseconds(500), [this]()
        {
          if (_runtime && _runtime->has_fatal_error()) {
            RCLCPP_ERROR(get_logger(), "vision runtime stopped: %s",
                       _runtime->fatal_error_message().c_str());
            rclcpp::shutdown();
          }
      });
  }

  ~VisionBridgeNode() override
  {
    if (_runtime) {
      _runtime->stop();
    }
  }

private:
  void publish_perf_summary()
  {
    if (!_runtime || !_perf_publisher) {
      return;
    }

    const auto snapshot = _runtime->telemetry_snapshot();
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(steady_now - _last_perf_time).count();

    double producer_fps = 0.0;
    double consumer_fps = 0.0;
    if (dt > 0.0) {
      producer_fps = static_cast<double>(snapshot.produced_count -
        _last_perf_snapshot.produced_count) / dt;
      consumer_fps = static_cast<double>(snapshot.consumed_count -
        _last_perf_snapshot.consumed_count) / dt;
    }

    omniseer_msgs::msg::VisionPerfSummary msg{};
    msg.header.stamp = static_cast<builtin_interfaces::msg::Time>(this->now());
    msg.header.frame_id = _config.camera_frame_id;
    msg.producer_fps = static_cast<float>(producer_fps);
    msg.consumer_fps = static_cast<float>(consumer_fps);
    msg.last_preprocess_ms = static_cast<float>(ns_to_ms(snapshot.last_preprocess_ns));
    msg.last_infer_ms = static_cast<float>(ns_to_ms(snapshot.last_infer_ns));
    msg.last_postprocess_ms = static_cast<float>(ns_to_ms(snapshot.last_postprocess_ns));
    msg.last_publish_ms = static_cast<float>(ns_to_ms(snapshot.last_publish_ns));
    msg.last_producer_total_ms = static_cast<float>(ns_to_ms(snapshot.last_producer_total_ns));
    msg.last_consumer_total_ms = static_cast<float>(ns_to_ms(snapshot.last_consumer_total_ns));
    msg.last_source_age_start_ms =
      static_cast<float>(ns_to_ms(snapshot.last_source_age_start_ns));
    msg.last_source_age_end_ms =
      static_cast<float>(ns_to_ms(snapshot.last_source_age_end_ns));
    msg.produced_count = snapshot.produced_count;
    msg.consumed_count = snapshot.consumed_count;
    msg.no_writable_buffer_count = snapshot.no_writable_buffer_count;
    msg.capture_retryable_error_count = snapshot.capture_retryable_error_count;
    msg.capture_fatal_error_count = snapshot.capture_fatal_error_count;
    msg.preprocess_error_count = snapshot.preprocess_error_count;
    msg.infer_error_count = snapshot.infer_error_count;
    _perf_publisher->publish(std::move(msg));

    _last_perf_time = steady_now;
    _last_perf_snapshot = snapshot;
  }

  VisionBridgeRuntimeConfig declare_bridge_parameters()
  {
    VisionBridgeRuntimeConfig cfg{};
    cfg.camera_device = declare_parameter<std::string>("camera.device", cfg.camera_device);
    cfg.camera_width = declare_parameter<int64_t>("camera.width", cfg.camera_width);
    cfg.camera_height = declare_parameter<int64_t>("camera.height", cfg.camera_height);
    cfg.camera_buffer_count =
      declare_parameter<int64_t>("camera.buffer_count", cfg.camera_buffer_count);

    cfg.pipeline_dst_width =
      declare_parameter<int64_t>("pipeline.dst_width", cfg.pipeline_dst_width);
    cfg.pipeline_dst_height =
      declare_parameter<int64_t>("pipeline.dst_height", cfg.pipeline_dst_height);

    cfg.detector_model_path =
      declare_parameter<std::string>("models.detector_model_path", cfg.detector_model_path);
    cfg.clip_model_path =
      declare_parameter<std::string>("models.clip_model_path", cfg.clip_model_path);
    cfg.clip_vocab_path =
      declare_parameter<std::string>("models.clip_vocab_path", cfg.clip_vocab_path);

    cfg.class_list_path =
      declare_parameter<std::string>("classes.path", cfg.class_list_path);
    cfg.pad_token =
      declare_parameter<std::string>("classes.pad_token", cfg.pad_token);

    cfg.producer_preflight_capture_wait_ms = declare_parameter<int64_t>(
          "producer.preflight_capture_wait_ms", cfg.producer_preflight_capture_wait_ms);
    cfg.runner_warmup_runs =
      declare_parameter<int64_t>("runner.warmup_runs", cfg.runner_warmup_runs);

    cfg.score_threshold =
      declare_parameter<double>("postprocess.score_threshold", cfg.score_threshold);
    cfg.nms_iou_threshold =
      declare_parameter<double>("postprocess.nms_iou_threshold", cfg.nms_iou_threshold);
    cfg.max_detections =
      declare_parameter<int64_t>("postprocess.max_detections", cfg.max_detections);

    cfg.camera_frame_id =
      declare_parameter<std::string>("frames.camera_frame_id", cfg.camera_frame_id);

    return cfg;
  }

  void log_config(const VisionBridgeRuntimeConfig & cfg) const
  {
    std::ostringstream message;
    message   << "configured bridge:"
              << " camera.device=" << cfg.camera_device
              << " camera.width=" << cfg.camera_width
              << " camera.height=" << cfg.camera_height
              << " camera.buffer_count=" << cfg.camera_buffer_count
              << " pipeline.dst_width=" << cfg.pipeline_dst_width
              << " pipeline.dst_height=" << cfg.pipeline_dst_height
              << " models.detector_model_path=" << quote_or_placeholder(cfg.detector_model_path)
              << " models.clip_model_path=" << quote_or_placeholder(cfg.clip_model_path)
              << " models.clip_vocab_path=" << quote_or_placeholder(cfg.clip_vocab_path)
              << " classes.path=" << quote_or_placeholder(cfg.class_list_path)
              << " classes.pad_token=" << quote_or_placeholder(cfg.pad_token)
              << " producer.preflight_capture_wait_ms=" << cfg.producer_preflight_capture_wait_ms
              << " runner.warmup_runs=" << cfg.runner_warmup_runs
              << " postprocess.score_threshold=" << cfg.score_threshold
              << " postprocess.nms_iou_threshold=" << cfg.nms_iou_threshold
              << " postprocess.max_detections=" << cfg.max_detections
              << " frames.camera_frame_id=" << quote_or_placeholder(cfg.camera_frame_id);
    RCLCPP_INFO(get_logger(), "%s", message.str().c_str());
  }

  static std::string quote_or_placeholder(const std::string & value)
  {
    if (value.empty()) {
      return "<unset>";
    }
    return "\"" + value + "\"";
  }

  VisionBridgeRuntimeConfig _config{};
  std::unique_ptr<VisionBridgeRuntime> _runtime{};
  rclcpp::Publisher<yolo_msgs::msg::DetectionArray>::SharedPtr _detections_publisher{};
  rclcpp::Publisher<omniseer_msgs::msg::VisionPerfSummary>::SharedPtr _perf_publisher{};
  std::unique_ptr<RosYoloDetectionsSink> _detections_sink{};
  rclcpp::TimerBase::SharedPtr _perf_timer{};
  rclcpp::TimerBase::SharedPtr _health_timer{};
  std::chrono::steady_clock::time_point _last_perf_time{};
  omniseer::vision::RollingTelemetrySnapshot _last_perf_snapshot{};
};
} // namespace omniseer_vision_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<omniseer_vision_bridge::VisionBridgeNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_bridge"), "fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
