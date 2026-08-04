#define OMNISEER_VISION_BRIDGE_TESTING
#include "../src/vision_bridge_node.cpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace omniseer_vision_bridge
{
namespace
{
using namespace std::chrono_literals;

class VisionBridgeNodeTest : public ::testing::Test
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

TEST_F(VisionBridgeNodeTest, DetectionSinkPublishesLatestOnlyQosAndNativeMetadata)
{
  const auto topic = "/vision_bridge_node_test/detections";
  auto publisher_node = std::make_shared<rclcpp::Node>("vision_bridge_node_test_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("vision_bridge_node_test_subscriber");
  auto publisher =
    publisher_node->create_publisher<yolo_msgs::msg::DetectionArray>(
      topic, latest_only_detection_qos());

  std::optional<yolo_msgs::msg::DetectionArray> received;
  auto subscription = subscriber_node->create_subscription<yolo_msgs::msg::DetectionArray>(
    topic, latest_only_detection_qos(),
    [&received](const yolo_msgs::msg::DetectionArray & msg)
    {
      received = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  ASSERT_TRUE(spin_until(executor, [&]() {return publisher->get_subscription_count() > 0;}));

  const auto publishers = subscriber_node->get_publishers_info_by_topic(topic);
  ASSERT_EQ(publishers.size(), 1U);
  const auto detection_qos = publishers.front().qos_profile().get_rmw_qos_profile();
  EXPECT_EQ(detection_qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(detection_qos.depth, 1U);
  EXPECT_EQ(detection_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(detection_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

  RosYoloDetectionsSink sink(publisher, {"person"}, "camera_optical_frame");
  omniseer::vision::DetectionsFrame frame{};
  frame.frame_id = 42;
  frame.sequence = 9001;
  frame.capture_ts_real_ns = 1234567890ULL;
  frame.count = 1;
  frame.detections[0].class_id = 0;
  frame.detections[0].score = 0.875F;
  frame.detections[0].x1 = 10.0F;
  frame.detections[0].y1 = 20.0F;
  frame.detections[0].x2 = 110.0F;
  frame.detections[0].y2 = 220.0F;

  sink.publish(frame);

  ASSERT_TRUE(spin_until(executor, [&]() {return received.has_value();}));
  EXPECT_EQ(received->header.frame_id, "camera_optical_frame");
  EXPECT_EQ(received->header.stamp.sec, 1);
  EXPECT_EQ(received->header.stamp.nanosec, 234567890U);
  EXPECT_EQ(received->native_frame_id, 42U);
  EXPECT_EQ(received->native_sequence, 9001U);
  ASSERT_EQ(received->detections.size(), 1U);
  EXPECT_EQ(received->detections.front().class_name, "person");
  EXPECT_DOUBLE_EQ(received->detections.front().score, 0.875);
  EXPECT_DOUBLE_EQ(received->detections.front().bbox.center.position.x, 60.0);
  EXPECT_DOUBLE_EQ(received->detections.front().bbox.center.position.y, 120.0);

  executor.remove_node(subscriber_node);
  executor.remove_node(publisher_node);
  subscription.reset();
}
} // namespace
} // namespace omniseer_vision_bridge
