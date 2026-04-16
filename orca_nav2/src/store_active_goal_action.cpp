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
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("stored_goal"),
      BT::OutputPort<bool>(
        "ice_measurement_stop",
        "True when input_goal.header.frame_id is map_ice_measurement (CSV up_down)."),
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
  const bool ice_measurement = (goal.header.frame_id == "map_ice_measurement");
  setOutput("stored_goal", goal);
  setOutput("ice_measurement_stop", ice_measurement);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::StoreActiveGoal>("StoreActiveGoal");
}