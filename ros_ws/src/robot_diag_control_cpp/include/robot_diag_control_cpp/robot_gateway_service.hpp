#pragma once

#include <grpcpp/grpcpp.h>

#include "robot_diag_control/api/robot_gateway.grpc.pb.h"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"

namespace robot_diag_control_cpp
{
class RobotGatewayService final : public omniseer::gateway::v1::RobotGateway::Service
{
public:
  explicit RobotGatewayService(GatewayStateStore & store, PreviewProcessManager & preview_manager);

  grpc::Status GetSystemStatus(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::GetSystemStatusRequest * request,
    omniseer::gateway::v1::SystemStatus * response) override;

  grpc::Status SetPreviewMode(
    grpc::ServerContext * context,
    const omniseer::gateway::v1::SetPreviewModeRequest * request,
    omniseer::gateway::v1::SetPreviewModeResponse * response) override;

private:
  GatewayStateStore & _store;
  PreviewProcessManager & _preview_manager;
};
} // namespace robot_diag_control_cpp
