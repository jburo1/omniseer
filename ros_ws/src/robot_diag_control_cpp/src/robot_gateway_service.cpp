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

gateway_proto::TeleopState to_proto(TeleopState state)
{
  switch (state) {
    case TeleopState::Disabled:
      return gateway_proto::TELEOP_DISABLED;
    case TeleopState::Enabled:
      return gateway_proto::TELEOP_ENABLED;
    case TeleopState::TimedOut:
      return gateway_proto::TELEOP_TIMED_OUT;
  }

  return gateway_proto::TELEOP_STATE_UNSPECIFIED;
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

gateway_proto::TeleopStatus to_proto(const TeleopStatusSnapshot & snapshot)
{
  gateway_proto::TeleopStatus response;
  response.set_state(to_proto(snapshot.state));
  response.set_enabled(snapshot.enabled);
  response.set_timed_out(snapshot.timed_out);
  response.set_last_command_age_ms(snapshot.last_command_age_ms);
  response.set_max_linear_mps(static_cast<float>(snapshot.max_linear_mps));
  response.set_max_angular_rad_s(static_cast<float>(snapshot.max_angular_rad_s));
  response.set_last_error(snapshot.last_error);
  return response;
}

gateway_proto::OverlayDetection to_proto(const DetectionOverlayItem & snapshot)
{
  gateway_proto::OverlayDetection response;
  response.set_class_id(snapshot.class_id);
  response.set_class_name(snapshot.class_name);
  response.set_score(static_cast<float>(snapshot.score));
  response.set_track_id(snapshot.track_id);
  response.set_bbox_center_x_px(static_cast<float>(snapshot.bbox_center_x_px));
  response.set_bbox_center_y_px(static_cast<float>(snapshot.bbox_center_y_px));
  response.set_bbox_width_px(static_cast<float>(snapshot.bbox_width_px));
  response.set_bbox_height_px(static_cast<float>(snapshot.bbox_height_px));
  return response;
}

gateway_proto::DetectionOverlayStatus to_proto(const DetectionOverlaySnapshot & snapshot)
{
  gateway_proto::DetectionOverlayStatus response;
  response.set_available(snapshot.available);
  response.set_stale(snapshot.stale);
  response.set_age_ms(snapshot.age_ms);
  response.set_source_width_px(snapshot.source_width_px);
  response.set_source_height_px(snapshot.source_height_px);
  response.set_detection_count(static_cast<uint32_t>(snapshot.detections.size()));
  for (const auto & detection : snapshot.detections) {
    *response.add_detections() = to_proto(detection);
  }
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
  *response.mutable_teleop() = to_proto(snapshot.teleop);
  return response;
}
} // namespace

RobotGatewayService::RobotGatewayService(
  GatewayStateStore & store, PreviewProcessManager & preview_manager)
: _store(store),
  _preview_manager(preview_manager)
{
}

RobotGatewayService::RobotGatewayService(
  GatewayStateStore & store, PreviewProcessManager & preview_manager,
  TeleopManager & teleop_manager)
: _store(store),
  _preview_manager(preview_manager),
  _teleop_manager(&teleop_manager)
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

grpc::Status RobotGatewayService::SetTeleopEnabled(
  grpc::ServerContext * context,
  const gateway_proto::SetTeleopEnabledRequest * request,
  gateway_proto::SetTeleopEnabledResponse * response)
{
  (void)context;

  if (_teleop_manager == nullptr) {
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "teleop is not configured");
  }

  const auto result = _teleop_manager->set_enabled(request->enabled());
  response->set_accepted(result.accepted);
  response->set_message(result.message);
  *response->mutable_teleop() = to_proto(result.teleop);
  return grpc::Status::OK;
}

grpc::Status RobotGatewayService::SendTeleopCommand(
  grpc::ServerContext * context,
  const gateway_proto::SendTeleopCommandRequest * request,
  gateway_proto::SendTeleopCommandResponse * response)
{
  (void)context;

  if (_teleop_manager == nullptr) {
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "teleop is not configured");
  }

  const auto result = _teleop_manager->send_command(
    TeleopCommand{
      request->linear_x_mps(),
      request->linear_y_mps(),
      request->angular_z_rad_s(),
    });
  response->set_accepted(result.accepted);
  response->set_message(result.message);
  *response->mutable_teleop() = to_proto(result.teleop);
  return grpc::Status::OK;
}

grpc::Status RobotGatewayService::GetOverlaySnapshot(
  grpc::ServerContext * context,
  const gateway_proto::GetOverlaySnapshotRequest * request,
  gateway_proto::OverlaySnapshot * response)
{
  (void)context;
  (void)request;
  _preview_manager.poll();
  *response->mutable_status() = to_proto(_store.get_system_status());
  *response->mutable_detections() = to_proto(_store.get_detection_overlay());
  return grpc::Status::OK;
}
} // namespace robot_diag_control_cpp
