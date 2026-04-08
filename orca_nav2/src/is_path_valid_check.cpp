#include "orca_nav2/is_path_valid_check.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "angles/angles.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// Path / distance math is duplicated from PurePursuitController3D (same formulas, no shared
// header). If you change tracking_error_from_plan or get_dist_L2_norm there, update here too.

namespace
{

double get_dist_l2_norm(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool transform_pose_to_frame(
  tf2_ros::Buffer & tf_buffer,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const std::string & target_frame,
  double transform_tolerance_sec)
{
  try {
    const auto tf = tf_buffer.lookupTransform(
      target_frame, in_pose.header.frame_id, in_pose.header.stamp,
      tf2::durationFromSec(transform_tolerance_sec));
    tf2::doTransform(in_pose, out_pose, tf);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void tracking_error_from_plan(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_in_path_frame,
  double & cross_track_xy_m,
  double & vertical_error_m,
  double & yaw_error_rad,
  geometry_msgs::msg::Point & closest_map)
{
  cross_track_xy_m = 0.0;
  vertical_error_m = 0.0;
  yaw_error_rad = 0.0;
  closest_map.x = 0.0;
  closest_map.y = 0.0;
  closest_map.z = 0.0;

  if (path.poses.empty()) {
    return;
  }

  const double rx = pose_in_path_frame.pose.position.x;
  const double ry = pose_in_path_frame.pose.position.y;
  const double rz = pose_in_path_frame.pose.position.z;
  const double robot_yaw = tf2::getYaw(pose_in_path_frame.pose.orientation);

  if (path.poses.size() == 1) {
    const auto & p = path.poses[0].pose.position;
    closest_map = p;
    const double dx = rx - p.x;
    const double dy = ry - p.y;
    const double path_yaw = tf2::getYaw(path.poses[0].pose.orientation);

    const double cp = std::cos(path_yaw) * dy - std::sin(path_yaw) * dx;
    const double sign = (cp >= 0.0) ? 1.0 : -1.0;

    cross_track_xy_m = sign * std::sqrt(dx * dx + dy * dy);
    vertical_error_m = rz - p.z;
    yaw_error_rad = angles::shortest_angular_distance(path_yaw, robot_yaw);
    return;
  }

  double best_dist_sq = std::numeric_limits<double>::infinity();
  double best_qx = rx;
  double best_qy = ry;
  double best_qz = rz;
  double best_path_yaw = tf2::getYaw(path.poses[0].pose.orientation);
  double best_sign = 1.0;

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i].pose.position;
    const auto & p2 = path.poses[i + 1].pose.position;
    const double vx = p2.x - p1.x;
    const double vy = p2.y - p1.y;
    const double wx = rx - p1.x;
    const double wy = ry - p1.y;
    const double vv = vx * vx + vy * vy;
    double t = 0.0;
    if (vv > 1e-12) {
      t = (wx * vx + wy * vy) / vv;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double qx = p1.x + t * vx;
    const double qy = p1.y + t * vy;
    const double qz = p1.z + t * (p2.z - p1.z);
    const double ex = rx - qx;
    const double ey = ry - qy;
    const double dist_sq = ex * ex + ey * ey;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_qx = qx;
      best_qy = qy;
      best_qz = qz;
      if (vv > 1e-12) {
        best_path_yaw = std::atan2(vy, vx);
        const double cp = vx * wy - vy * wx;
        best_sign = (cp >= 0.0) ? 1.0 : -1.0;
      } else {
        best_path_yaw = tf2::getYaw(path.poses[i].pose.orientation);
        const double cp = std::cos(best_path_yaw) * wy - std::sin(best_path_yaw) * wx;
        best_sign = (cp >= 0.0) ? 1.0 : -1.0;
      }
    }
  }

  if (std::isfinite(best_dist_sq)) {
    cross_track_xy_m = best_sign * std::sqrt(best_dist_sq);
    vertical_error_m = rz - best_qz;
    yaw_error_rad = angles::shortest_angular_distance(best_path_yaw, robot_yaw);
    closest_map.x = best_qx;
    closest_map.y = best_qy;
    closest_map.z = best_qz;
  }
}

}  // namespace

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
  std::string global_frame = "map";
  std::string robot_base_frame = "base_link";
  double transform_tolerance = 1.0;

  if (!getInput("path", path) || !getInput("max_dist", max_dist)) {
    return BT::NodeStatus::FAILURE;
  }
  static_cast<void>(getInput("goal_tolerance_m", goal_tolerance_m));
  if (path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("goal", goal)) {
    return BT::NodeStatus::FAILURE;
  }
  if (!path.header.frame_id.empty() && !goal.header.frame_id.empty() &&
    path.header.frame_id != goal.header.frame_id)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (get_dist_l2_norm(path.poses.back().pose.position, goal.pose.position) > goal_tolerance_m) {
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("tf_buffer", tf_buffer)) {
    return BT::NodeStatus::FAILURE;
  }

  if (auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node")) {
    if (node->has_parameter("global_frame")) {
      global_frame = node->get_parameter("global_frame").as_string();
    }
    if (node->has_parameter("robot_base_frame")) {
      robot_base_frame = node->get_parameter("robot_base_frame").as_string();
    }
    if (node->has_parameter("transform_tolerance")) {
      transform_tolerance = node->get_parameter("transform_tolerance").as_double();
    }
  }

  if (!nav2_util::getCurrentPose(
      pose, *tf_buffer, global_frame, robot_base_frame, transform_tolerance))
  {
    return BT::NodeStatus::FAILURE;
  }

  const std::string plan_frame =
    path.header.frame_id.empty() ? global_frame : path.header.frame_id;
  geometry_msgs::msg::PoseStamped pose_in_path_frame = pose;
  if (pose_in_path_frame.header.frame_id != plan_frame) {
    if (!transform_pose_to_frame(
        *tf_buffer, pose, pose_in_path_frame, plan_frame, transform_tolerance))
    {
      return BT::NodeStatus::FAILURE;
    }
  }

  tracking_error_from_plan(
    path, pose_in_path_frame, cross_track_xy_m, vertical_error_m, yaw_error_rad, closest_map);
  static_cast<void>(closest_map);

  if (std::abs(cross_track_xy_m) > max_dist || std::abs(vertical_error_m) > max_dist) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsPathValidCheck>("OrcaGeomIsPathValid");
}
