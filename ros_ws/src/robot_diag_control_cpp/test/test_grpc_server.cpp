#include <chrono>
#include <memory>

#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "robot_diag_control/api/robot_gateway.grpc.pb.h"
#include "robot_diag_control_cpp/preview_command_factory.hpp"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/grpc_server.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"
#include "robot_diag_control_cpp/robot_gateway_service.hpp"

namespace robot_diag_control_cpp
{
namespace
{
namespace gateway_proto = omniseer::gateway::v1;

TEST(RobotGatewayGrpcTest, ServesSystemStatusAndPreviewUpdatesOverGrpc)
{
  GatewayStateStore store;
  PreviewProcessManager preview_manager(
    store,
    make_fixed_preview_command_factory(PreviewProcessCommand{"/bin/sleep", {"30"}}),
    std::chrono::milliseconds(250));
  omniseer_msgs::msg::VisionPerfSummary msg{};
  msg.producer_fps = 22.0F;
  msg.consumer_fps = 21.5F;
  msg.last_infer_ms = 6.5F;
  msg.infer_error_count = 1;
  msg.capture_fatal_error_count = 0;
  store.update_vision_perf(msg);
  nav_msgs::msg::Odometry odom{};
  odom.twist.twist.linear.x = 0.3;
  odom.twist.twist.linear.y = 0.4;
  odom.twist.twist.angular.z = 0.2;
  store.update_odometry(odom);

  RobotGatewayService service(store, preview_manager);
  GrpcServer server(service, "127.0.0.1", 0);
  server.start();

  const auto channel = grpc::CreateChannel(
    server.listen_address(), grpc::InsecureChannelCredentials());
  ASSERT_TRUE(channel->WaitForConnected(
      std::chrono::system_clock::now() + std::chrono::seconds(2)));

  auto stub = gateway_proto::RobotGateway::NewStub(channel);

  gateway_proto::SystemStatus system_status;
  gateway_proto::GetSystemStatusRequest system_request;
  grpc::ClientContext system_context;
  const auto system_rpc_status =
    stub->GetSystemStatus(&system_context, system_request, &system_status);
  ASSERT_TRUE(system_rpc_status.ok());
  EXPECT_EQ(system_status.gateway_name(), "robot_diag_control_cpp");
  EXPECT_EQ(system_status.preview().state(), gateway_proto::PREVIEW_DISABLED);
  EXPECT_EQ(system_status.preview().profile(), gateway_proto::PREVIEW_PROFILE_BALANCED);
  EXPECT_TRUE(system_status.vision().available());
  EXPECT_FALSE(system_status.vision().stale());
  EXPECT_EQ(system_status.health().state(), gateway_proto::ROBOT_HEALTH_OK);
  EXPECT_TRUE(system_status.health().ready());
  EXPECT_TRUE(system_status.health().odom_available());
  EXPECT_FALSE(system_status.health().odom_stale());
  EXPECT_EQ(system_status.health().summary(), "robot healthy");
  EXPECT_FLOAT_EQ(system_status.health().linear_speed_mps(), 0.5F);
  EXPECT_FLOAT_EQ(system_status.health().angular_speed_rad_s(), 0.2F);
  EXPECT_FLOAT_EQ(system_status.vision().producer_fps(), 22.0F);
  EXPECT_FLOAT_EQ(system_status.vision().consumer_fps(), 21.5F);
  EXPECT_FLOAT_EQ(system_status.vision().last_infer_ms(), 6.5F);
  EXPECT_EQ(system_status.vision().infer_error_count(), 1U);

  gateway_proto::SetPreviewModeRequest preview_request;
  preview_request.set_enabled(true);
  preview_request.set_profile(gateway_proto::PREVIEW_PROFILE_LOW_BW);

  gateway_proto::SetPreviewModeResponse preview_response;
  grpc::ClientContext preview_context;
  const auto preview_rpc_status =
    stub->SetPreviewMode(&preview_context, preview_request, &preview_response);
  ASSERT_TRUE(preview_rpc_status.ok());
  EXPECT_TRUE(preview_response.accepted());
  EXPECT_EQ(preview_response.message(), "preview started");
  EXPECT_EQ(preview_response.preview().state(), gateway_proto::PREVIEW_RUNNING);
  EXPECT_EQ(preview_response.preview().profile(), gateway_proto::PREVIEW_PROFILE_LOW_BW);

  gateway_proto::SystemStatus updated_status;
  grpc::ClientContext updated_context;
  const auto updated_rpc_status =
    stub->GetSystemStatus(&updated_context, system_request, &updated_status);
  ASSERT_TRUE(updated_rpc_status.ok());
  EXPECT_EQ(updated_status.preview().state(), gateway_proto::PREVIEW_RUNNING);
  EXPECT_EQ(updated_status.preview().profile(), gateway_proto::PREVIEW_PROFILE_LOW_BW);

  gateway_proto::SetPreviewModeRequest disable_request;
  disable_request.set_enabled(false);
  gateway_proto::SetPreviewModeResponse disable_response;
  grpc::ClientContext disable_context;
  const auto disable_rpc_status =
    stub->SetPreviewMode(&disable_context, disable_request, &disable_response);
  ASSERT_TRUE(disable_rpc_status.ok());
  EXPECT_TRUE(disable_response.accepted());
  EXPECT_EQ(disable_response.message(), "preview stopped");
  EXPECT_EQ(disable_response.preview().state(), gateway_proto::PREVIEW_DISABLED);
}

TEST(RobotGatewayGrpcTest, RejectsUnknownPreviewProfileWhenEnabling)
{
  GatewayStateStore store;
  PreviewProcessManager preview_manager(store);
  RobotGatewayService service(store, preview_manager);
  GrpcServer server(service, "127.0.0.1", 0);
  server.start();

  const auto channel = grpc::CreateChannel(
    server.listen_address(), grpc::InsecureChannelCredentials());
  ASSERT_TRUE(channel->WaitForConnected(
      std::chrono::system_clock::now() + std::chrono::seconds(2)));

  auto stub = gateway_proto::RobotGateway::NewStub(channel);

  gateway_proto::SetPreviewModeRequest request;
  request.set_enabled(true);
  request.set_profile(static_cast<gateway_proto::PreviewProfile>(999));

  gateway_proto::SetPreviewModeResponse response;
  grpc::ClientContext context;
  const auto rpc_status = stub->SetPreviewMode(&context, request, &response);
  EXPECT_EQ(rpc_status.error_code(), grpc::StatusCode::INVALID_ARGUMENT);
  EXPECT_EQ(rpc_status.error_message(), "unsupported preview profile");
}

TEST(RobotGatewayGrpcTest, ReturnsRejectedResponseWhenPreviewCommandIsUnset)
{
  GatewayStateStore store;
  PreviewProcessManager preview_manager(store);
  RobotGatewayService service(store, preview_manager);
  GrpcServer server(service, "127.0.0.1", 0);
  server.start();

  const auto channel = grpc::CreateChannel(
    server.listen_address(), grpc::InsecureChannelCredentials());
  ASSERT_TRUE(channel->WaitForConnected(
      std::chrono::system_clock::now() + std::chrono::seconds(2)));

  auto stub = gateway_proto::RobotGateway::NewStub(channel);

  gateway_proto::SetPreviewModeRequest request;
  request.set_enabled(true);
  request.set_profile(gateway_proto::PREVIEW_PROFILE_BALANCED);

  gateway_proto::SetPreviewModeResponse response;
  grpc::ClientContext context;
  const auto rpc_status = stub->SetPreviewMode(&context, request, &response);
  ASSERT_TRUE(rpc_status.ok());
  EXPECT_FALSE(response.accepted());
  EXPECT_EQ(response.message(), "preview command not configured");
  EXPECT_EQ(response.preview().state(), gateway_proto::PREVIEW_DISABLED);
  EXPECT_EQ(response.preview().last_error(), "preview command not configured");
}
} // namespace
} // namespace robot_diag_control_cpp
