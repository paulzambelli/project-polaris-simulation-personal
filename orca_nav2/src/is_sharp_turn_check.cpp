#include "orca_nav2/is_sharp_turn_check.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"
#include <cmath>


namespace orca_nav2
{

    IsSharpTurnCheck::IsSharpTurnCheck(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
    {
    }

    BT::NodeStatus IsSharpTurnCheck::tick()
    {
        nav_msgs::msg::Path path;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        geometry_msgs::msg::PoseStamped pose;   
        geometry_msgs::msg::Point closest_map;
        double min_angle_rad; //45 degree
        double release_angle_rad; //5 degrees

        double cross_track_xy_m;
        double vertical_error_m;
        double yaw_error_rad;
        double transform_tolerance = 1.0;

        if (!getInput("path", path) || !getInput("min_angle_rad", min_angle_rad)) {
            return BT::NodeStatus::FAILURE;
        }
        else if (!config().blackboard->get("tf_buffer", tf_buffer))
        {
            return BT::NodeStatus::FAILURE;
        }

        if (!nav2_util::getCurrentPose(pose, *tf_buffer, "map", "base_link", transform_tolerance)) {
            return BT::NodeStatus::FAILURE; 
        }

        tracking_errors_along_path(path, pose, cross_track_xy_m, vertical_error_m, yaw_error_rad, closest_map);
        double abs_yaw_error_rad = std::abs(yaw_error_rad);

        // Like Schmitt-trigger behaviour.

        //Start turning
        if (!is_turning_) {
            if (abs_yaw_error_rad > min_angle_rad) {
                is_turning_ = true;
            }
        } else {
            if (abs_yaw_error_rad < release_angle_rad) {
                is_turning_ = false;
            } //Should end turning
        }

        return is_turning_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
} // namespace orca_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<orca_nav2::IsSharpTurnCheck>("IsSharpTurn");
}