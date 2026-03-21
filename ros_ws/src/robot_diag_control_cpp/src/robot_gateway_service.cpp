#include "robot_diag_control_cpp/robot_gateway_service.hpp"

#include <optional>

namespace robot_diag_control_cpp
{
namespace
{
namespace gateway_proto = omniseer::gateway::v1;

gateway_proto::PreviewState to_proto(PreviewState state)
{
  switch (state) {
    case PreviewState::Disabled:
      return gateway_proto::PREVIEW_DISABLED;
    case PreviewState::Running:
      return gateway_proto::PREVIEW_RUNNING;
  }

  return gateway_proto::PREVIEW_STATE_UNSPECIFIED;
}

gateway_proto::PreviewProfile to_proto(PreviewProfile profile)
{
  switch (profile) {
    case PreviewProfile::LowBw:
      return gateway_proto::PREVIEW_PROFILE_LOW_BW;
    case PreviewProfile::Balanced:
      return gateway_proto::PREVIEW_PROFILE_BALANCED;
    case PreviewProfile::HighQuality:
      return gateway_proto::PREVIEW_PROFILE_HIGH_QUALITY;
  }

  return gateway_proto::PREVIEW_PROFILE_UNSPECIFIED;
}

gateway_proto::RobotHealthState to_proto(RobotHealthState state)
{
  switch (state) {
    case RobotHealthState::Ok:
      return gateway_proto::ROBOT_HEALTH_OK;
    case RobotHealthState::Degraded:
      return gateway_proto::ROBOT_HEALTH_DEGRADED;
  }

  return gateway_proto::ROBOT_HEALTH_STATE_UNSPECIFIED;
}

std::optional<PreviewProfile> from_proto(gateway_proto::PreviewProfile profile)
{
  switch (profile) {
    case gateway_proto::PREVIEW_PROFILE_UNSPECIFIED:
      return std::nullopt;
    case gateway_proto::PREVIEW_PROFILE_LOW_BW:
      return PreviewProfile::LowBw;
    case gateway_proto::PREVIEW_PROFILE_BALANCED:
      return PreviewProfile::Balanced;
    case gateway_proto::PREVIEW_PROFILE_HIGH_QUALITY:
      return PreviewProfile::HighQuality;
    default:
      return std::nullopt;
  }
}

gateway_proto::PreviewStatus to_proto(const PreviewStatusSnapshot & snapshot)
{
  gateway_proto::PreviewStatus response;
  response.set_state(to_proto(snapshot.state));
  response.set_profile(to_proto(snapshot.profile));
  response.set_last_error(snapshot.last_error);
  return response;
}

gateway_proto::VisionStatus to_proto(const VisionStatusSnapshot & snapshot)
{
  gateway_proto::VisionStatus response;
  response.set_available(snapshot.available);
  response.set_stale(snapshot.stale);
  response.set_producer_fps(static_cast<float>(snapshot.producer_fps));
  response.set_consumer_fps(static_cast<float>(snapshot.consumer_fps));
  response.set_last_infer_ms(static_cast<float>(snapshot.last_infer_ms));
  response.set_infer_error_count(snapshot.infer_error_count);
  response.set_capture_fatal_error_count(snapshot.capture_fatal_error_count);
  return response;
}

gateway_proto::RobotHealth to_proto(const RobotHealthSnapshot & snapshot)
{
  gateway_proto::RobotHealth response;
  response.set_state(to_proto(snapshot.state));
  response.set_ready(snapshot.ready);
  response.set_summary(snapshot.summary);
  response.set_odom_available(snapshot.odom_available);
  response.set_odom_stale(snapshot.odom_stale);
  response.set_linear_speed_mps(static_cast<float>(snapshot.linear_speed_mps));
  response.set_angular_speed_rad_s(static_cast<float>(snapshot.angular_speed_rad_s));
  return response;
}

gateway_proto::SystemStatus to_proto(const SystemStatusSnapshot & snapshot)
{
  gateway_proto::SystemStatus response;
  response.set_gateway_name(snapshot.gateway_name);
  response.set_gateway_version(snapshot.gateway_version);
  *response.mutable_preview() = to_proto(snapshot.preview);
  *response.mutable_vision() = to_proto(snapshot.vision);
  *response.mutable_health() = to_proto(snapshot.health);
  return response;
}
} // namespace

RobotGatewayService::RobotGatewayService(
  GatewayStateStore & store, PreviewProcessManager & preview_manager)
: _store(store),
  _preview_manager(preview_manager)
{
}

grpc::Status RobotGatewayService::GetSystemStatus(
  grpc::ServerContext * context,
  const gateway_proto::GetSystemStatusRequest * request,
  gateway_proto::SystemStatus * response)
{
  (void)context;
  (void)request;
  _preview_manager.poll();
  *response = to_proto(_store.get_system_status());
  return grpc::Status::OK;
}

grpc::Status RobotGatewayService::SetPreviewMode(
  grpc::ServerContext * context,
  const gateway_proto::SetPreviewModeRequest * request,
  gateway_proto::SetPreviewModeResponse * response)
{
  (void)context;

  const auto requested_profile = request->profile();
  const auto profile = from_proto(requested_profile);
  if (request->enabled() &&
    requested_profile != gateway_proto::PREVIEW_PROFILE_UNSPECIFIED &&
    !profile.has_value())
  {
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "unsupported preview profile");
  }

  const auto result = _preview_manager.set_enabled(request->enabled(), profile);
  response->set_accepted(result.accepted);
  response->set_message(result.message);
  *response->mutable_preview() = to_proto(result.preview);
  return grpc::Status::OK;
}
} // namespace robot_diag_control_cpp
