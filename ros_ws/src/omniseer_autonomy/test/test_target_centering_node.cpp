#define OMNISEER_AUTONOMY_TESTING
#include "../src/target_centering_node.cpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "omniseer_msgs/srv/capture_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace omniseer_autonomy
{
namespace
{
using namespace std::chrono_literals;

std::filesystem::path make_run_dir()
{
  auto path = std::filesystem::temp_directory_path() /
    ("omniseer-autonomy-node-test-" +
    std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path;
}

yolo_msgs::msg::DetectionArray centered_detection()
{
  yolo_msgs::msg::DetectionArray msg{};
  yolo_msgs::msg::Detection detection{};
  detection.class_name = "backpack";
  detection.score = 0.9;
  detection.bbox.center.position.x = 50.0;
  detection.bbox.center.position.y = 50.0;
  detection.bbox.size.x = 30.0;
  detection.bbox.size.y = 30.0;
  msg.detections.push_back(detection);
  return msg;
}

std::string read_text(const std::filesystem::path & path)
{
  std::ifstream input(path);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

template<typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor & executor, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(20ms);
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}
} // namespace

class TargetCenteringNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TargetCenteringNodeTest, WritesEventsAndRequestsCaptureOnSuccess)
{
  const auto run_dir = make_run_dir();
  const auto capture_service = "/target_centering_node_test/capture_frame";
  const auto detections_topic = "/target_centering_node_test/detections";
  const auto command_topic = "/target_centering_node_test/cmd_vel_autonomy";

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("target_class", "backpack"),
      rclcpp::Parameter("run_dir", run_dir.string()),
      rclcpp::Parameter("capture_service", capture_service),
      rclcpp::Parameter("detections_topic", detections_topic),
      rclcpp::Parameter("command_topic", command_topic),
      rclcpp::Parameter("image_width_px", 100.0),
      rclcpp::Parameter("image_height_px", 100.0),
      rclcpp::Parameter("stable_framed_frames", 1),
      rclcpp::Parameter("bbox_area_min_ratio", 0.05),
      rclcpp::Parameter("bbox_area_max_ratio", 0.20),
      rclcpp::Parameter("detection_stale_ms", 2000),
      rclcpp::Parameter("shutdown_on_terminal", false),
    });

  auto node = std::make_shared<TargetCenteringNode>(options);
  auto helper = std::make_shared<rclcpp::Node>("target_centering_node_test_helper");

  bool capture_seen = false;
  auto capture_server = helper->create_service<omniseer_msgs::srv::CaptureFrame>(
    capture_service,
    [&capture_seen](
      const std::shared_ptr<omniseer_msgs::srv::CaptureFrame::Request> request,
      std::shared_ptr<omniseer_msgs::srv::CaptureFrame::Response> response)
    {
      capture_seen = true;
      EXPECT_EQ(request->capture_reason, "target_framed");
      EXPECT_EQ(request->target_class, "backpack");
      EXPECT_NEAR(request->normalized_error, 0.0, 1.0e-9);
      EXPECT_NEAR(request->bbox_area_ratio, 0.09, 1.0e-9);
      response->success = true;
      response->reason = "saved";
      response->image_path = "/tmp/target.jpg";
      response->frame_id = 42;
      response->sequence = 7;
    });
  auto detections = helper->create_publisher<yolo_msgs::msg::DetectionArray>(detections_topic, 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper);

  ASSERT_TRUE(spin_until(executor, [&]() {return detections->get_subscription_count() > 0;}));
  for (int i = 0; i < 4; ++i) {
    detections->publish(centered_detection());
    executor.spin_some(50ms);
    std::this_thread::sleep_for(50ms);
  }

  const auto log_path = run_dir / "autonomy.jsonl";
  ASSERT_TRUE(spin_until(
    executor,
      [&]()
      {
        return capture_seen && std::filesystem::exists(log_path) &&
               read_text(log_path).find("\"event\":\"capture_result\"") != std::string::npos;
      }));

  const auto events = read_text(log_path);
  EXPECT_NE(events.find("\"event\":\"succeeded\""), std::string::npos);
  EXPECT_NE(events.find("\"event\":\"capture_result\""), std::string::npos);
  EXPECT_NE(events.find("\"success\":true"), std::string::npos);
  EXPECT_NE(events.find("\"image_path\":\"/tmp/target.jpg\""), std::string::npos);

  executor.remove_node(helper);
  executor.remove_node(node);
  capture_server.reset();
  std::filesystem::remove_all(run_dir);
}
} // namespace omniseer_autonomy
