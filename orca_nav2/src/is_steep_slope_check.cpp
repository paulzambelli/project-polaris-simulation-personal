#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "tf2_ros/buffer.h"

namespace
{
rclcpp::Clock g_log_throttle_clock(RCL_STEADY_TIME);
}

namespace orca_nav2
{

class IsSteepSlopeCheck : public BT::ConditionNode
{
public:
    IsSteepSlopeCheck(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
            BT::InputPort<double>("min_slope", 0.7, "Slope to trigger the Yes SteepLine"),
            BT::InputPort<double>("release_slope", 0.15, "Slope to release the SteepLine"),
            BT::InputPort<double>("max_xy_distance", 7.0, "Minimum horizontal distance to goal."),
        };
    }

    BT::NodeStatus tick() override;

private:
    bool is_replaning_;
    bool prev_is_replaning_;
};

IsSteepSlopeCheck::IsSteepSlopeCheck(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), is_replaning_(false), prev_is_replaning_(false)
{
}

BT::NodeStatus IsSteepSlopeCheck::tick()
{
    nav_msgs::msg::Path path;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Point closest_map;

    double min_slope;
    double release_slope;
    double max_xy_distance;
    double transform_tolerance = 1.0;

    auto logger = rclcpp::get_logger("bt_is_steep_line");

    if (!getInput("path", path) || !getInput("min_slope", min_slope) ||
        !getInput("release_slope", release_slope) || !getInput("max_xy_distance", max_xy_distance))
    {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "[IsSteepLine]: missing path or angle ports (check BT XML blackboard wiring)");
        return BT::NodeStatus::FAILURE;
    }
    if (path.poses.empty()) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000, "[IsSteepLine]: path has no poses yet");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer)) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "[IsSteepLine]: blackboard has no tf_buffer (Nav2 should inject this)");
        return BT::NodeStatus::FAILURE;
    }

    if (!nav2_util::getCurrentPose(pose, *tf_buffer, "map", "base_link", transform_tolerance)) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "[IsSteepLine]: getCurrentPose(map -> base_link) failed; "
            "condition fails and BT falls back to LongStraightLine");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_pos = path.poses.back().pose.position;
    auto robot_pos = pose.pose.position;

    double dx = goal_pos.x - robot_pos.x;
    double dy = goal_pos.y - robot_pos.y;
    double dz = std::abs(goal_pos.z - robot_pos.z); // Absolute vertical distance

    double dxy = std::sqrt(dx * dx + dy * dy); // Horizontal distance

    double current_slope;
    // Protect against division by zero if the robot is exactly on the XY target
    if (dxy < 0.001) {
        current_slope = std::numeric_limits<double>::max(); 
    } else {
        current_slope = dz / dxy;
    }

    // 3. Schmitt-trigger: hysteresis between Steep and PurePursuit branches
    // To ENTER steep mode, the slope must be HIGHER than min_slope (e.g., > 3.0)
    // To EXIT steep mode, the slope must drop BELOW release_slope (e.g., < 2.0)
    if (!is_replaning_) {
        if (current_slope > min_slope && dxy < max_xy_distance ) {
            is_replaning_ = true;
        }
    } else {
        if (current_slope < release_slope || dxy > max_xy_distance) {
            is_replaning_ = false;
        }
    }

    // 4. Logging
    if (is_replaning_ != prev_is_replaning_) {
        if (is_replaning_) {
            RCLCPP_INFO(
                logger,
                "[IsSteepLine]: SteepDescent Cont.: ON (slope=%.2f, threshold > %.2f)",
                current_slope, min_slope);
        } else {
            RCLCPP_INFO(
                logger,
                "[IsSteepLine]: PurePursuit Cont.: ON (slope=%.2f, release < %.2f)",
                current_slope, release_slope);
        }
        prev_is_replaning_ = is_replaning_;
    }

    return is_replaning_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

}

}  // namespace orca_nav2


BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<orca_nav2::IsSteepSlopeCheck>("IsSteepSlopeCheck");
}