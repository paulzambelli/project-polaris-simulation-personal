#include <string>
#include <memory>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace orca_nav2
{

class IsIceMeasurementStop : public BT::ConditionNode
{
public:
  IsIceMeasurementStop(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current waypoint goal to check for the map_ice_station flag."),
      BT::InputPort<bool>(
        "ice_measurement_stop", false,
        "From StoreActiveGoal when waypoint was marked ice (CSV up_down); survives if frame_id is rewritten."),
    };
  }

  BT::NodeStatus tick() override;
};


IsIceMeasurementStop::IsIceMeasurementStop(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}


BT::NodeStatus IsIceMeasurementStop::tick()
{
  bool ice_from_store = false;
  getInput("ice_measurement_stop", ice_from_store);

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }
  if (ice_from_store || goal.header.frame_id == "map_ice_measurement") {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsIceMeasurementStop>("IsIceMeasurementStop");
}

