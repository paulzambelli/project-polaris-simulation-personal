#ifndef ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_
#define ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include <cmath>

namespace orca_nav2
{

class IsPathValidCheck : public BT::ConditionNode
{
    public:
        IsPathValidCheck(
            const std::string & condition_name,
            const BT::NodeConfiguration & conf);
        

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
                BT::InputPort<double>("max_dist", 0.5, "Maximum allowed distance from path.")

            };
        }

        BT::NodeStatus tick() override;
};

} // namespace orca_nav2

#endif  // ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_