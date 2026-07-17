#pragma once

#include <grpcpp/grpcpp.h>

#include "robot_diag_control/api/robot_gateway.grpc.pb.h"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"
#include "robot_diag_control_cpp/teleop_manager.hpp"

namespace robot_diag_control_cpp
{
class RobotGatewayService final : public omniseer::gateway::v1::RobotGateway::Service
{
public:
  explicit RobotGatewayService(GatewayStateStore & store, PreviewProcessManager & preview_manager);
  RobotGatewayService(
    GatewayStateStore & store, PreviewProcessManager & preview_manager,
    TeleopManager & teleop_manager);

  grpc::Status GetSystemStatus(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::GetSystemStatusRequest * request,
    omniseer::gateway::v1::SystemStatus * response) override;

  grpc::Status SetPreviewMode(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::SetPreviewModeRequest * request,
    omniseer::gateway::v1::SetPreviewModeResponse * response) override;

  grpc::Status SetTeleopEnabled(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::SetTeleopEnabledRequest * request,
    omniseer::gateway::v1::SetTeleopEnabledResponse * response) override;

  grpc::Status SendTeleopCommand(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::SendTeleopCommandRequest * request,
    omniseer::gateway::v1::SendTeleopCommandResponse * response) override;

private:
  GatewayStateStore & _store;
  PreviewProcessManager & _preview_manager;
  TeleopManager * _teleop_manager{nullptr};
};
} // namespace robot_diag_control_cpp
