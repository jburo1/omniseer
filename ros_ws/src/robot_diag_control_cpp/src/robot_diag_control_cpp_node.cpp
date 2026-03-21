#include <memory>
#include <stdexcept>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/grpc_server.hpp"
#include "robot_diag_control_cpp/preview_command_factory.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"
#include "robot_diag_control_cpp/robot_gateway_service.hpp"

namespace robot_diag_control_cpp
{
class RobotDiagControlCppNode : public rclcpp::Node
{
public:
  RobotDiagControlCppNode()
  : rclcpp::Node("robot_diag_control_cpp")
  {
    const auto vision_stale_after_ms =
      declare_parameter<int64_t>("vision_stale_after_ms", 2000);
    const auto odom_stale_after_ms =
      declare_parameter<int64_t>("odom_stale_after_ms", 1000);
    const auto grpc_bind_address =
      declare_parameter<std::string>("grpc_bind_address", "0.0.0.0");
    const auto grpc_port = declare_parameter<int64_t>("grpc_port", 50051);
    const auto robot_health_odom_topic =
      declare_parameter<std::string>("robot_health_odom_topic", "/odometry/filtered");
    const auto preview_command = declare_parameter<std::string>("preview_command", "");
    const auto preview_args =
      declare_parameter<std::vector<std::string>>("preview_args", std::vector<std::string>{});
    const auto preview_source_kind = declare_parameter<std::string>(
      "preview_source_kind", "camera");
    const auto preview_device = declare_parameter<std::string>("preview_device", "/dev/video11");
    const auto preview_srt_bind_address =
      declare_parameter<std::string>("preview_srt_bind_address", "0.0.0.0");
    const auto preview_srt_port = declare_parameter<int64_t>("preview_srt_port", 7001);
    const auto preview_srt_latency_ms = declare_parameter<int64_t>("preview_srt_latency_ms", 125);
    if (grpc_port < 0 || grpc_port > 65535) {
      throw std::runtime_error("grpc_port must be between 0 and 65535");
    }
    if (preview_srt_port < 0 || preview_srt_port > 65535) {
      throw std::runtime_error("preview_srt_port must be between 0 and 65535");
    }
    if (preview_srt_latency_ms < 0) {
      throw std::runtime_error("preview_srt_latency_ms must be non-negative");
    }
    if (odom_stale_after_ms < 0) {
      throw std::runtime_error("odom_stale_after_ms must be non-negative");
    }

    _state_store = std::make_unique<GatewayStateStore>(
      get_name(),
      "0.1.0",
      std::chrono::milliseconds(vision_stale_after_ms),
      std::chrono::milliseconds(odom_stale_after_ms));
    PreviewCommandFactory preview_command_factory;
    if (!preview_command.empty()) {
      preview_command_factory = make_fixed_preview_command_factory(
        PreviewProcessCommand{preview_command, preview_args});
    } else {
      preview_command_factory = make_gstreamer_preview_command_factory(
        GstreamerPreviewConfig{
          "gst-launch-1.0",
          preview_source_kind,
          preview_device,
          preview_srt_bind_address,
          static_cast<int>(preview_srt_port),
          static_cast<int>(preview_srt_latency_ms),
        });
    }
    _preview_manager = std::make_unique<PreviewProcessManager>(
      *_state_store, std::move(preview_command_factory));
    _grpc_service = std::make_unique<RobotGatewayService>(*_state_store, *_preview_manager);
    _grpc_server = std::make_unique<GrpcServer>(
      *_grpc_service, grpc_bind_address, static_cast<int>(grpc_port));
    _grpc_server->start();

    _vision_perf_subscription = create_subscription<omniseer_msgs::msg::VisionPerfSummary>(
      "/vision/perf", 10,
      [this](const omniseer_msgs::msg::VisionPerfSummary & msg)
      {
        _state_store->update_vision_perf(msg);
      });
    _odometry_subscription = create_subscription<nav_msgs::msg::Odometry>(
      robot_health_odom_topic, 10,
      [this](const nav_msgs::msg::Odometry & msg)
      {
        _state_store->update_odometry(msg);
      });
    _preview_poll_timer = create_wall_timer(
      std::chrono::milliseconds(250),
      [this]()
      {
        _preview_manager->poll();
      });

    RCLCPP_INFO(
      get_logger(),
      "C++ gateway node started; gRPC listening on %s, /vision/perf aggregation is active, robot health odom topic is %s, preview source is %s",
      _grpc_server->listen_address().c_str(),
      robot_health_odom_topic.c_str(),
      preview_command.empty() ? preview_source_kind.c_str() : preview_command.c_str());
  }

  ~RobotDiagControlCppNode() override
  {
    if (_grpc_server) {
      _grpc_server->stop();
    }
  }

private:
  std::unique_ptr<GatewayStateStore> _state_store{};
  std::unique_ptr<PreviewProcessManager> _preview_manager{};
  std::unique_ptr<RobotGatewayService> _grpc_service{};
  std::unique_ptr<GrpcServer> _grpc_server{};
  rclcpp::Subscription<omniseer_msgs::msg::VisionPerfSummary>::SharedPtr
    _vision_perf_subscription{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscription{};
  rclcpp::TimerBase::SharedPtr _preview_poll_timer{};
};
} // namespace robot_diag_control_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_diag_control_cpp::RobotDiagControlCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("robot_diag_control_cpp"),
      "failed to start C++ gateway node: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
}
