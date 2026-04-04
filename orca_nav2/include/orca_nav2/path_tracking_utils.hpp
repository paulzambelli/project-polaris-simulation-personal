// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Shared path geometry: L2 distance, pure-pursuit lookahead goal, cross-track / vertical / yaw errors.

#ifndef ORCA_NAV2__PATH_TRACKING_UTILS_HPP_
#define ORCA_NAV2__PATH_TRACKING_UTILS_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace orca_nav2
{

/** Euclidean distance between two 3D points. */
double point_distance_l2(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2);

/**
 * Pure-pursuit style goal: walk the plan until distance to the robot stops decreasing, then return
 * the first pose farther than lookahead_dist from the robot (L2 in XYZ), or the last pose.
 */
geometry_msgs::msg::PoseStamped find_pure_pursuit_goal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_f_map,
  double lookahead_dist);

/**
 * Tracking errors in path.header.frame_id: signed XY cross-track (m), vertical error (m),
 * yaw error (rad), and the closest point on the path polyline (map frame).
 */
void tracking_errors_along_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_f_map,
  double & cross_track_xy_m,
  double & vertical_error_m,
  double & yaw_error_rad,
  geometry_msgs::msg::Point & closest_map);

}  // namespace orca_nav2

#endif  // ORCA_NAV2__PATH_TRACKING_UTILS_HPP_
