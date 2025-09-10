#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace omniseer
{

  class SelectFrontierFromCostmap : public BT::SyncActionNode
  {
  public:
    SelectFrontierFromCostmap(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
    }

    BT::NodeStatus tick() override
    {
      // Placeholder: no selection logic yet.
      return BT::NodeStatus::FAILURE;
    }
  };

} // namespace omniseer

// Register with BT factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<omniseer::SelectFrontierFromCostmap>("SelectFrontierFromCostmap");
}
