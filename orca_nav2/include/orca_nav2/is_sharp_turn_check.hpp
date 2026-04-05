#ifndef ORCA_NAV2__IS_SHARP_TURN_CHECK_HPP_
#define ORCA_NAV2__IS_SHARP_TURN_CHECK_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include <cmath>

namespace orca_nav2
{

class IsSharpTurnCheck : public BT::ConditionNode
{
    public:
        IsSharpTurnCheck(
            const std::string & condition_name,
            const BT::NodeConfiguration & conf) : BT::ConditionNode(name, config), is_turning_(false) {}
        

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
                BT::InputPort<nav_msgs::msg::Path>("min_angle_rad", "Angle to trigger the Yes SharpTurn"),
                BT::InputPort<nav_msgs::msg::Path>("release_angle_rad", "Angle to release the SharpTurn"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        bool is_turning_;
};

} // namespace orca_nav2

#endif  // ORCA_NAV2__IS_SHARP_TURN_CHECK_HPP_