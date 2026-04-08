#include "orca_nav2/is_path_valid_check.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"
#include <cmath>


namespace orca_nav2
{

    IsPathValidCheck::IsPathValidCheck(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
    {
    }

    BT::NodeStatus IsPathValidCheck::tick()
    {
        nav_msgs::msg::Path path;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        geometry_msgs::msg::PoseStamped pose;   
        geometry_msgs::msg::Point closest_map;
        double max_dist;
        double goal_tolerance_m = 0.75;
        geometry_msgs::msg::PoseStamped goal;
        double cross_track_xy_m;
        double vertical_error_m;
        double yaw_error_rad;
        double transform_tolerance = 1.0;

        if (!getInput("path", path) || !getInput("max_dist", max_dist)) {
            return BT::NodeStatus::FAILURE;
        }
        static_cast<void>(getInput("goal_tolerance_m", goal_tolerance_m));
        if (path.poses.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        // NavigateToPose updates blackboard "goal" on each waypoint but does not clear "path".
        // A geometrically "valid" stale path would skip ComputePathToPose and FollowPath would
        // track the previous goal — breaks waypoint_follower chains.
        if (!config().blackboard->get("goal", goal)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!path.header.frame_id.empty() && !goal.header.frame_id.empty() &&
            path.header.frame_id != goal.header.frame_id)
        {
            return BT::NodeStatus::FAILURE;
        }
        if (point_distance_l2(path.poses.back().pose.position, goal.pose.position) > goal_tolerance_m) {
            return BT::NodeStatus::FAILURE;
        }
        if (!config().blackboard->get("tf_buffer", tf_buffer))
        {
            return BT::NodeStatus::FAILURE;
        }

        if (!nav2_util::getCurrentPose(pose, *tf_buffer, "map", "base_link", transform_tolerance)) {
            return BT::NodeStatus::FAILURE; 
        }

        tracking_errors_along_path(
            path, pose, cross_track_xy_m, vertical_error_m, yaw_error_rad, closest_map);
        
        if (std::abs(cross_track_xy_m) > max_dist || std::abs(vertical_error_m) > max_dist)  {
            return BT::NodeStatus::FAILURE;
        } else {
            return BT::NodeStatus::SUCCESS;
        }

    }
} // namespace orca_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<orca_nav2::IsPathValidCheck>("IsPathValid");
}