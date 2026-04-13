#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace orca_nav2
{

class StoreActiveGoal : public BT::SyncActionNode
{
public:
  StoreActiveGoal(
    const std::string& name, 
    const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts() 
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("stored_goal")
    };
  }

  BT::NodeStatus tick() override;
};

StoreActiveGoal::StoreActiveGoal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus StoreActiveGoal::tick() 
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("input_goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }
  // "Pin" the data to the new key
  setOutput("stored_goal", goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::StoreActiveGoal>("StoreActiveGoal");
}