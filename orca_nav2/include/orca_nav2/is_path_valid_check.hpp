#ifndef ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_
#define ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
                BT::InputPort<double>(
                    "max_dist", 0.5,
                    "Max |cross-track XY| and |vertical error| vs path (same definitions as "
                    "PurePursuitController3D; logic lives only in is_path_valid_check.cpp)."),
                BT::InputPort<double>(
                    "goal_tolerance_m", 0.75,
                    "If path end is farther than this from blackboard goal, path is stale (replan)."),
            };
        }

        BT::NodeStatus tick() override;
};

} // namespace orca_nav2

#endif  // ORCA_NAV2__IS_PATH_VALID_CHECK_HPP_