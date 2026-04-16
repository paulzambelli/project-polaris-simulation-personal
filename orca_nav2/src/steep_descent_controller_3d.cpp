// SteepDescentController3D
//
// Used when the path slope |ΔZ| / ΔXY exceeds a threshold — i.e., the goal is nearly directly
// above or below the robot. In that case PurePursuitController3D spirals because it tries to
// correct XY cross-track while also commanding Z, which fights itself. This controller simply
// zeros linear.x and angular.z and only commands linear.z toward the goal depth.

#include <cassert>
#include <cmath>
#include <memory>
#include <string>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "orca_nav2/param_macro.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace orca_nav2
{

namespace
{
// Local copy to avoid ODR issues with the other controllers in the same shared library.
class Limiter
{
  double max_a_{};
  double max_dv_{};

public:
  Limiter() = default;

  Limiter(const double & max_a, const double & dt)
  : max_a_{max_a}, max_dv_{max_a * dt}
  {
    assert(max_a > 0);
    assert(dt > 0);
  }

  // Reduce |v| so the robot can stop within goal_dist at max_a. v and goal_dist must have the
  // same sign (both positive or both negative).
  void decelerate(double & v, const double & goal_dist) const
  {
    assert(v * goal_dist >= 0);
    const double decel_v = std::sqrt(2.0 * std::abs(goal_dist) * max_a_);
    v = std::copysign(std::min(std::abs(v), decel_v), v);
  }

  // Clamp the change in v to ±max_dv per tick.
  void limit(double & v, const double & prev_v) const
  {
    const double dv = v - prev_v;
    if (dv > max_dv_) {
      v = prev_v + max_dv_;
    } else if (dv < -max_dv_) {
      v = prev_v - max_dv_;
    }
  }
};
}  // namespace

class SteepDescentController3D : public nav2_core::Controller
{
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string base_frame_id_;

  // Parameters
  double z_vel_{};
  double z_accel_{};
  double goal_tolerance_{};
  double tick_rate_{};
  double transform_tolerance_{};

  // Plan from StraightLinePlanner3D (map frame)
  nav_msgs::msg::Path plan_;

  // Previous command for acceleration limiting
  double prev_z_vel_{0.0};

  Limiter z_limiter_;

  bool publish_tracking_error_{true};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr vertical_error_pub_;

  bool transform_pose(
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const std::string & target_frame) const
  {
    try {
      const auto transform = tf_->lookupTransform(
        target_frame, in_pose.header.frame_id,
        in_pose.header.stamp,
        tf2::durationFromSec(transform_tolerance_));
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(logger_, "SteepDescent: transform failed: %s", ex.what());
      return false;
    }
  }

public:
  SteepDescentController3D() = default;
  ~SteepDescentController3D() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto parent = weak_parent.lock();
    logger_ = parent->get_logger();
    tf_ = tf;
    base_frame_id_ = costmap_ros->getBaseFrameID();

    PARAMETER(parent, name, z_vel, 0.1)
    PARAMETER(parent, name, z_accel, 0.1)
    PARAMETER(parent, name, goal_tolerance, 0.2)
    PARAMETER(parent, name, tick_rate, 20.0)
    PARAMETER(parent, name, transform_tolerance, 1.0)
    PARAMETER(parent, name, publish_tracking_error, true)

    z_limiter_ = Limiter(z_accel_, 1.0 / tick_rate_);

    if (publish_tracking_error_) {
      vertical_error_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
        "steep_descent_vertical_error", rclcpp::QoS(10));
    }

    RCLCPP_INFO(logger_, "SteepDescentController3D configured (z_vel=%.2f, z_accel=%.2f)",
      z_vel_, z_accel_);
  }

  void cleanup() override
  {
    vertical_error_pub_.reset();
  }

  void activate() override
  {
    if (vertical_error_pub_) {
      vertical_error_pub_->on_activate();
    }
  }

  void deactivate() override
  {
    if (vertical_error_pub_) {
      vertical_error_pub_->on_deactivate();
    }
  }

  // pose is base_link in odom frame (from nav2 controller server)
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    // Default: all zeros — safe stop if anything goes wrong.

    if (plan_.poses.empty()) {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "SteepDescent: plan is empty, commanding zero velocity");
      return cmd_vel;
    }

    // Transform robot pose into the plan's frame (map).
    geometry_msgs::msg::PoseStamped pose_in_map;
    if (!transform_pose(pose, pose_in_map, plan_.header.frame_id)) {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "SteepDescent: cannot transform pose to map frame, commanding zero velocity");
      return cmd_vel;
    }

    // Z goal is the last pose in the plan (goal waypoint depth).
    const double goal_z = plan_.poses.back().pose.position.z;
    const double robot_z = pose_in_map.pose.position.z;
    const double z_error = goal_z - robot_z;  // positive = need to go up, negative = need to go down

    if (publish_tracking_error_ && vertical_error_pub_) {
      std_msgs::msg::Float64 msg;
      msg.data = z_error;
      vertical_error_pub_->publish(msg);
    }

    // Only command Z if outside goal tolerance.
    if (std::abs(z_error) > goal_tolerance_) {
      cmd_vel.twist.linear.z = std::copysign(z_vel_, z_error);
      z_limiter_.decelerate(cmd_vel.twist.linear.z, z_error);
    }

    // Apply acceleration limit relative to previous command.
    z_limiter_.limit(cmd_vel.twist.linear.z, prev_z_vel_);
    prev_z_vel_ = cmd_vel.twist.linear.z;

    // Explicitly zero out XY — do not spiral.
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    return cmd_vel;
  }

  void setPlan(const nav_msgs::msg::Path & plan) override
  {
    if (plan.poses.empty()) {
      throw nav2_core::PlannerException("SteepDescent: received plan with zero length");
    }
    plan_ = plan;
    prev_z_vel_ = 0.0;  // reset on new plan so accel limit starts fresh
  }

  void setSpeedLimit(const double &, const bool &) override
  {
    RCLCPP_WARN(logger_, "SteepDescentController3D: setSpeedLimit not supported");
  }
};

}  // namespace orca_nav2

PLUGINLIB_EXPORT_CLASS(orca_nav2::SteepDescentController3D, nav2_core::Controller)
