#include <cmath>
#include <gtest/gtest.h>
#include <vector>

#include "omniseer_autonomy/target_centering_controller.hpp"

namespace omniseer_autonomy
{
  namespace
  {
    constexpr double kPi = 3.14159265358979323846;

    TargetCenteringConfig config()
    {
      TargetCenteringConfig out{};
      out.target_class            = "backpack";
      out.image_width_px          = 100.0;
      out.image_height_px         = 100.0;
      out.scan_yaw_rate_rad_s     = 0.2;
      out.max_yaw_rate_rad_s      = 0.3;
      out.min_yaw_rate_rad_s      = 0.08;
      out.kp                      = 0.3;
      out.center_deadband         = 0.05;
      out.bbox_area_min_ratio     = 0.08;
      out.bbox_area_max_ratio     = 0.35;
      out.forward_speed_m_s       = 0.05;
      out.reverse_speed_m_s       = 0.04;
      out.stable_framed_frames    = 3;
      out.proximity_stop_m        = 0.3;
      out.detection_stale_sec     = 0.5;
      out.scan_limit_revolutions  = 1.0;
      out.target_lost_timeout_sec = 0.5;
      return out;
    }

    TargetDetection detection(double center_x, double confidence = 0.8,
                              std::string class_name = "backpack", double size_x = 30.0,
                              double size_y = 30.0, double center_y = 50.0)
    {
      return TargetDetection{class_name, confidence, center_x, center_y, size_x, size_y};
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

  TEST(TargetCenteringController, WaitsForFirstDetectionMessageBeyondStaleThreshold)
  {
    TargetCenteringController controller(config());

    const auto before_perception = controller.tick(0.6);

    EXPECT_EQ(controller.state(), CenteringState::Scan);
    EXPECT_TRUE(before_perception.publish_command);
    EXPECT_DOUBLE_EQ(before_perception.angular_z_rad_s, 0.0);
    EXPECT_TRUE(controller.terminal_reason().empty());
  }

  TEST(TargetCenteringController, RequiresThreeConsecutiveDetectionsBeforeAcquisition)
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
    EXPECT_DOUBLE_EQ(acquired.linear_x_m_s, 0.0);
  }

  TEST(TargetCenteringController, LowConfidenceDetectionsDoNotContributeToAcquisition)
  {
    TargetCenteringController controller(config());

    controller.update_detections({detection(20.0, 0.49)}, 0.1);
    controller.update_detections({detection(20.0)}, 0.2);
    controller.update_detections({detection(20.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Scan);
    controller.update_detections({detection(20.0)}, 0.4);
    EXPECT_EQ(controller.state(), CenteringState::Center);
  }

  TEST(TargetCenteringController, DiscontinuousDetectionsRestartAcquisition)
  {
    TargetCenteringController controller(config());

    controller.update_detections({detection(20.0)}, 0.1);
    controller.update_detections({detection(80.0)}, 0.2);
    controller.update_detections({detection(80.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Scan);
    controller.update_detections({detection(80.0)}, 0.4);
    EXPECT_EQ(controller.state(), CenteringState::Center);
  }

  TEST(TargetCenteringController, ThreeContinuousQualifyingDetectionsAcquireTarget)
  {
    TargetCenteringController controller(config());

    controller.update_detections({detection(20.0)}, 0.1);
    controller.update_detections({detection(25.0)}, 0.2);
    controller.update_detections({detection(30.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Center);
  }

  TEST(TargetCenteringController, RejectsLargeTargetJumpAfterLock)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(20.0)}, 0.1);
    controller.update_detections({detection(20.0)}, 0.2);
    controller.update_detections({detection(20.0)}, 0.3);

    const auto output = controller.update_detections({detection(80.0)}, 0.4);

    EXPECT_EQ(controller.state(), CenteringState::Center);
    EXPECT_EQ(controller.target_loss_count(), 1);
    EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, NearbyQualifyingTargetMaintainsLock)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(20.0)}, 0.1);
    controller.update_detections({detection(20.0)}, 0.2);
    controller.update_detections({detection(20.0)}, 0.3);

    const auto output = controller.update_detections({detection(25.0)}, 0.4);

    EXPECT_EQ(controller.state(), CenteringState::Center);
    EXPECT_EQ(controller.target_loss_count(), 0);
    EXPECT_GT(output.angular_z_rad_s, 0.0);
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
    cfg.kp   = 0.01;
    TargetCenteringController controller(cfg);
    controller.update_detections({detection(55.0)}, 0.1);
    controller.update_detections({detection(55.0)}, 0.2);

    const auto output = controller.update_detections({detection(55.0)}, 0.3);

    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, -cfg.min_yaw_rate_rad_s);
  }

  TEST(TargetCenteringController, SucceedsAfterStableFramedFrames)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0)}, 0.1);
    controller.update_detections({detection(50.0)}, 0.2);
    controller.update_detections({detection(50.0)}, 0.3);
    controller.update_detections({detection(50.0)}, 0.4);

    const auto output = controller.update_detections({detection(50.0)}, 0.5);

    EXPECT_EQ(controller.state(), CenteringState::Success);
    EXPECT_EQ(controller.terminal_reason(), "framed");
    EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
    ASSERT_TRUE(controller.time_to_centered_sec().has_value());
    EXPECT_DOUBLE_EQ(*controller.final_error(), 0.0);
    EXPECT_DOUBLE_EQ(*controller.final_confidence(), 0.8);
  }

  TEST(TargetCenteringController, TicksDoNotAdvanceStableFramedDetections)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0)}, 0.1);
    controller.update_detections({detection(50.0)}, 0.2);
    controller.update_detections({detection(50.0)}, 0.3);

    for (double time = 0.31; time < 0.50; time += 0.01)
    {
      const auto output = controller.tick(time);
      EXPECT_TRUE(output.publish_command);
      EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
      EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
      EXPECT_EQ(controller.state(), CenteringState::Frame);
    }

    controller.update_detections({detection(50.0)}, 0.50);
    EXPECT_EQ(controller.state(), CenteringState::Frame);
    controller.update_detections({detection(50.0)}, 0.60);
    EXPECT_EQ(controller.state(), CenteringState::Success);
  }

  TEST(TargetCenteringController, RequiresConfiguredFreshFramedDetectionsForSuccess)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0)}, 0.1);
    controller.update_detections({detection(50.0)}, 0.2);
    controller.update_detections({detection(50.0)}, 0.3);
    EXPECT_EQ(controller.state(), CenteringState::Frame);
    controller.update_detections({detection(50.0)}, 0.4);
    EXPECT_EQ(controller.state(), CenteringState::Frame);
    controller.update_detections({detection(50.0)}, 0.5);
    EXPECT_EQ(controller.state(), CenteringState::Success);
  }

  TEST(TargetCenteringController, FreshMissingOrInvalidTargetResetsFramedStability)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0)}, 0.1);
    controller.update_detections({detection(50.0)}, 0.2);
    controller.update_detections({detection(50.0)}, 0.3);
    controller.update_detections({}, 0.4);
    controller.update_detections({detection(80.0)}, 0.5);
    controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.6);

    controller.update_detections({detection(50.0)}, 0.7);
    controller.update_detections({detection(50.0)}, 0.8);
    EXPECT_EQ(controller.state(), CenteringState::Frame);
    controller.update_detections({detection(50.0)}, 0.9);
    EXPECT_EQ(controller.state(), CenteringState::Success);
  }

  TEST(TargetCenteringController, TickPublishesCachedTargetCommandWithoutVisualTransition)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(80.0)}, 0.1);
    controller.update_detections({detection(80.0)}, 0.2);
    controller.update_detections({detection(80.0)}, 0.3);

    const auto output = controller.tick(0.35);

    EXPECT_TRUE(output.publish_command);
    EXPECT_LT(output.angular_z_rad_s, 0.0);
    EXPECT_EQ(controller.state(), CenteringState::Center);
  }

  TEST(TargetCenteringController, TooSmallTargetCommandsForward)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.1);
    controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.2);

    const auto output =
        controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Frame);
    EXPECT_DOUBLE_EQ(output.linear_x_m_s, config().forward_speed_m_s);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, TooLargeTargetCommandsBackward)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(50.0, 0.8, "backpack", 70.0, 70.0)}, 0.1);
    controller.update_detections({detection(50.0, 0.8, "backpack", 70.0, 70.0)}, 0.2);

    const auto output =
        controller.update_detections({detection(50.0, 0.8, "backpack", 70.0, 70.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Frame);
    EXPECT_DOUBLE_EQ(output.linear_x_m_s, -config().reverse_speed_m_s);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, YawCorrectionContinuesWhileMovingForward)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(80.0, 0.8, "backpack", 10.0, 10.0)}, 0.1);
    controller.update_detections({detection(80.0, 0.8, "backpack", 10.0, 10.0)}, 0.2);

    const auto output =
        controller.update_detections({detection(80.0, 0.8, "backpack", 10.0, 10.0)}, 0.3);

    EXPECT_GT(output.linear_x_m_s, 0.0);
    EXPECT_LT(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, YawCorrectionContinuesWhileMovingBackward)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(20.0, 0.8, "backpack", 70.0, 70.0)}, 0.1);
    controller.update_detections({detection(20.0, 0.8, "backpack", 70.0, 70.0)}, 0.2);

    const auto output =
        controller.update_detections({detection(20.0, 0.8, "backpack", 70.0, 70.0)}, 0.3);

    EXPECT_LT(output.linear_x_m_s, 0.0);
    EXPECT_GT(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, FailsStoppedWhenProximityBlocksRequiredForwardMotion)
  {
    TargetCenteringController controller(config());
    controller.update_proximity_range(0.2);
    controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.1);
    controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.2);

    const auto output =
        controller.update_detections({detection(50.0, 0.8, "backpack", 10.0, 10.0)}, 0.3);

    EXPECT_EQ(controller.state(), CenteringState::Failed);
    EXPECT_EQ(controller.terminal_reason(), "proximity_blocked");
    EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, RejectsInvalidContinuityConfiguration)
  {
    auto invalid_confidence                  = config();
    invalid_confidence.min_target_confidence = 1.01;
    EXPECT_THROW(TargetCenteringController{invalid_confidence}, std::invalid_argument);

    auto invalid_jump                         = config();
    invalid_jump.max_target_center_jump_ratio = 0.0;
    EXPECT_THROW(TargetCenteringController{invalid_jump}, std::invalid_argument);
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

  TEST(TargetCenteringController, FailsAfterOneScanRevolution)
  {
    auto cfg                   = config();
    cfg.scan_limit_revolutions = 1.0;
    cfg.detection_stale_sec    = 2.0;
    TargetCenteringController controller(cfg);
    controller.update_detections({}, 0.0);
    controller.update_heading(0.0);
    controller.update_heading(kPi);
    controller.update_heading(0.0);

    const auto output = controller.tick(0.2);

    EXPECT_EQ(controller.state(), CenteringState::Failed);
    EXPECT_EQ(controller.terminal_reason(), "scan_complete_no_target");
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, 0.0);
  }

  TEST(TargetCenteringController, ReacquiresAfterTargetLostTimeout)
  {
    TargetCenteringController controller(config());
    controller.update_detections({detection(20.0)}, 0.0);
    controller.update_detections({detection(20.0)}, 0.1);
    controller.update_detections({detection(20.0)}, 0.2);
    controller.update_detections({}, 0.3);

    const auto output = controller.tick(0.75);

    EXPECT_EQ(controller.state(), CenteringState::Scan);
    EXPECT_TRUE(controller.terminal_reason().empty());
    EXPECT_EQ(controller.target_loss_count(), 1);
    EXPECT_DOUBLE_EQ(output.angular_z_rad_s, config().scan_yaw_rate_rad_s);
    ASSERT_GE(output.events.size(), 2U);
    EXPECT_EQ(output.events.front().event, "reacquire_started");
    EXPECT_EQ(output.events.front().reason, "target_lost_timeout");

    controller.update_detections({detection(50.0)}, 0.8);
    controller.update_detections({detection(50.0)}, 0.9);

    const auto reacquired = controller.update_detections({detection(50.0)}, 1.0);

    EXPECT_EQ(controller.state(), CenteringState::Frame);
    EXPECT_EQ(controller.terminal_reason(), "");
    EXPECT_DOUBLE_EQ(reacquired.angular_z_rad_s, 0.0);
  }
} // namespace omniseer_autonomy
