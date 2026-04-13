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
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Current waypoint goal to check for the map_ice_station flag."
      ),
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
  geometry_msgs::msg::PoseStamped goal;

  // Here better to have the default as SUCCESS, so that if the topic is not available
  // to robot will not drive fully into the ice.

  if (!getInput("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }
  if (goal.header.frame_id == "map_ice_measurement") {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsIceMeasurementStop>("IsIceMeasurementStop");
}

