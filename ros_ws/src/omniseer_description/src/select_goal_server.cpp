#include <ament_index_cpp/get_package_prefix.hpp>
#include <atomic>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

namespace
{
  constexpr const char* kPluginLibrary = "libomniseer_bt_nodes.so";
  constexpr const char* kTreeXml       = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <SelectExplorationGoal
      global_frame="map"
      robot_frame="base_link"
      costmap_service="/global_costmap/get_costmap"
      planner_action="/planner_server/compute_path_to_pose"
      top_k="8"
      ig_max_range_m="6.0"
      ig_num_rays="48"
      ig_fov_rad="6.28318530718"
      use_nav2_plan_cost="true"
      costmap_timeout_ms="1000"
      planner_timeout_ms="2000"
      debug_enabled="true"
      goal="{goal}"/>
  </BehaviorTree>
</root>)";
} // namespace

class SelectGoalServer : public rclcpp::Node
{
public:
  SelectGoalServer() : rclcpp::Node("select_goal_server"), busy_(false)
  {
    try
    {
      this->declare_parameter("use_sim_time", true);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      // Already declared by launch or overrides; ignore
    }
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr)
        {
          if (busy_.exchange(true))
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "select_goal_server already processing a request");
            return;
          }
          std::thread(
              [self = shared_from_this()]()
              {
                auto       node    = std::static_pointer_cast<SelectGoalServer>(self);
                const bool success = node->runOnce();
                RCLCPP_INFO(node->get_logger(), "SelectExplorationGoal %s",
                            success ? "SUCCESS" : "FAILURE");
                node->busy_.store(false);
              })
              .detach();
        });

    trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/omniseer/select_goal/run",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          if (busy_.exchange(true))
          {
            response->success = false;
            response->message = "SelectExplorationGoal request already in progress";
            return;
          }

          const bool success = runOnce();
          busy_.store(false);

          response->success = success;
          response->message = success ? "OK" : "FAILED";
        });

    RCLCPP_INFO(
        get_logger(),
        "select_goal_server ready. Use RViz Publish Point or call /omniseer/select_goal/run");
  }

private:
  bool runOnce()
  try
  {
    const std::string pkg_prefix  = ament_index_cpp::get_package_prefix("omniseer_description");
    const std::string plugin_path = pkg_prefix + "/lib/" + kPluginLibrary;

    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(plugin_path);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", std::static_pointer_cast<rclcpp::Node>(shared_from_this()));

    auto tree = factory.createTreeFromText(kTreeXml, blackboard);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate   rate(50.0);

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      status = tree.tickOnce();
      // rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }

    return status == BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception running SelectExplorationGoal: %s", e.what());
    return false;
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                trigger_srv_;
  std::atomic<bool>                                                 busy_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelectGoalServer>());
  rclcpp::shutdown();
  return 0;
}
