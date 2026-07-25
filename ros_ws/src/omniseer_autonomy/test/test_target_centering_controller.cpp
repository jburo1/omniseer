#include <gtest/gtest.h>

#include <vector>

#include "omniseer_autonomy/target_centering_controller.hpp"

namespace omniseer_autonomy
{
namespace
{
TargetCenteringConfig config()
{
  TargetCenteringConfig out{};
  out.target_class = "backpack";
  out.image_width_px = 100.0;
  out.scan_yaw_rate_rad_s = 0.2;
  out.max_yaw_rate_rad_s = 0.3;
  out.min_yaw_rate_rad_s = 0.08;
  out.kp = 0.3;
  out.center_deadband = 0.05;
  out.stable_center_frames = 3;
  out.detection_stale_sec = 0.5;
  out.scan_timeout_sec = 12.0;
  out.target_lost_timeout_sec = 0.5;
  return out;
}

TargetDetection detection(
  double center_x, double confidence = 0.8,
  std::string class_name = "backpack")
{
  return TargetDetection{class_name, confidence, center_x, 20.0, 10.0, 10.0};
}
} // namespace

TEST(TargetCenteringController, ScansOnlyAfterFreshDetectionMessages)
{
  TargetCenteringController controller(config());

  const auto before_perception = controller.tick(0.1);
  EXPECT_TRUE(before_perception.publish_command);
  EXPECT_DOUBLE_EQ(before_perception.angular_z_rad_s, 0.0);
  EXPECT_EQ(controller.state(), CenteringState::Scan);

  const auto with_empty_frame = controller.update_detections({}, 0.2);
  EXPECT_TRUE(with_empty_frame.publish_command);
  EXPECT_DOUBLE_EQ(with_empty_frame.angular_z_rad_s, 0.2);
}

TEST(TargetCenteringController, RequiresThreeOfFiveFramesBeforeAcquisition)
{
  TargetCenteringController controller(config());

  EXPECT_EQ(controller.update_detections({detection(20.0)}, 0.1).angular_z_rad_s, 0.2);
  EXPECT_EQ(controller.update_detections({}, 0.2).angular_z_rad_s, 0.2);
  EXPECT_EQ(controller.update_detections({detection(20.0)}, 0.3).angular_z_rad_s, 0.2);
  EXPECT_EQ(controller.state(), CenteringState::Scan);

  const auto acquired = controller.update_detections({detection(20.0)}, 0.4);

  EXPECT_EQ(controller.state(), CenteringState::Center);
  EXPECT_TRUE(acquired.publish_command);
  EXPECT_GT(acquired.events.size(), 1U);
  EXPECT_GT(acquired.angular_z_rad_s, 0.0);
}

TEST(TargetCenteringController, UsesNegativeYawForTargetRightOfImageCenter)
{
  TargetCenteringController controller(config());
  controller.update_detections({detection(80.0)}, 0.1);
  controller.update_detections({detection(80.0)}, 0.2);

  const auto output = controller.update_detections({detection(80.0)}, 0.3);

  EXPECT_EQ(controller.state(), CenteringState::Center);
  EXPECT_LT(output.angular_z_rad_s, 0.0);
}

TEST(TargetCenteringController, AppliesMinimumUsefulYawOutsideDeadband)
{
  auto cfg = config();
  cfg.kp = 0.01;
  TargetCenteringController controller(cfg);
  controller.update_detections({detection(55.0)}, 0.1);
  controller.update_detections({detection(55.0)}, 0.2);

  const auto output = controller.update_detections({detection(55.0)}, 0.3);

  EXPECT_DOUBLE_EQ(output.angular_z_rad_s, -cfg.min_yaw_rate_rad_s);
}

TEST(TargetCenteringController, SucceedsAfterStableCenteredFrames)
{
  TargetCenteringController controller(config());
  controller.update_detections({detection(50.0)}, 0.1);
  controller.update_detections({detection(50.0)}, 0.2);
  controller.update_detections({detection(50.0)}, 0.3);
  controller.update_detections({detection(50.0)}, 0.4);

  const auto output = controller.update_detections({detection(50.0)}, 0.5);

  EXPECT_EQ(controller.state(), CenteringState::Success);
  EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  ASSERT_TRUE(controller.time_to_centered_sec().has_value());
  EXPECT_DOUBLE_EQ(*controller.final_error(), 0.0);
  EXPECT_DOUBLE_EQ(*controller.final_confidence(), 0.8);
}

TEST(TargetCenteringController, FailsOnStaleDetectionsAndCommandsZero)
{
  TargetCenteringController controller(config());
  controller.update_detections({}, 0.0);

  const auto output = controller.tick(0.6);

  EXPECT_EQ(controller.state(), CenteringState::Failed);
  EXPECT_EQ(controller.terminal_reason(), "stale_detections");
  EXPECT_TRUE(output.publish_command);
  EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
}

TEST(TargetCenteringController, FailsOnScanTimeout)
{
  auto cfg = config();
  cfg.scan_timeout_sec = 1.0;
  cfg.detection_stale_sec = 2.0;
  TargetCenteringController controller(cfg);
  controller.update_detections({}, 0.0);

  const auto output = controller.tick(1.1);

  EXPECT_EQ(controller.state(), CenteringState::Failed);
  EXPECT_EQ(controller.terminal_reason(), "scan_timeout");
  EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
}

TEST(TargetCenteringController, FailsAfterTargetLostTimeout)
{
  TargetCenteringController controller(config());
  controller.update_detections({detection(20.0)}, 0.0);
  controller.update_detections({detection(20.0)}, 0.1);
  controller.update_detections({detection(20.0)}, 0.2);
  controller.update_detections({}, 0.3);

  const auto output = controller.tick(0.75);

  EXPECT_EQ(controller.state(), CenteringState::Failed);
  EXPECT_EQ(controller.terminal_reason(), "target_lost_timeout");
  EXPECT_EQ(controller.target_loss_count(), 1);
  EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
}
} // namespace omniseer_autonomy
