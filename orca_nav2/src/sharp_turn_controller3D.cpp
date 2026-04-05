#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <cassert>

#include "orca_nav2/param_macro.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "orca_nav2/sharp_turn_controller3D.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_nav2
{
  constexpr bool sign(const double &v) { return v > 0; }

  Limiter::Limiter(const double &max_a, const double &dt)
      : max_a_{max_a}, max_dv_{max_a * dt}
  {
    assert(max_a > 0);
    assert(dt > 0);
  }

  void Limiter::decelerate(double &v, const double &goal_dist) const
    {
      assert(v * goal_dist >= 0);
      auto decel_v = std::sqrt(2 * std::abs(goal_dist) * max_a_);
      auto result_v = std::min(std::abs(v), decel_v);
      v = std::copysign(result_v, v);
    }

    // Limit implementation
    void Limiter::limit(double &v, const double &prev_v) const
    {
      auto dv = v - prev_v;
      if (dv > max_dv_)
      {
        v = prev_v + max_dv_;
      }
      else if (dv < -max_dv_)
      {
        v = prev_v - max_dv_;
      }
    }


  class SharpTurnController3D : public nav2_core::Controller
  {
    rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string base_frame_id_;

    // Parameters
    double z_vel_{};
    double z_accel_{};
    double yaw_vel_{};
    double yaw_accel_{};
    double lookahead_dist_{};
    double transform_tolerance_{};
    double goal_tolerance_{}; // Stop motion when we're very close to the goal
    double tick_rate_{};      // Tick rate, used to compute dt

    rclcpp::Duration transform_tolerance_d_{0, 0};
    // Plan from StraightLinePlanner3D
    nav_msgs::msg::Path plan_;
    // Keep track of the previous cmd_vel to limit acceleration
    geometry_msgs::msg::Twist prev_vel_{};

    Limiter z_limiter_;
    Limiter yaw_limiter_;

    double yaw_error_{};

    bool publish_tracking_error_{true};
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr cross_track_xy_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr vertical_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_map_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_twist_pub_;

    static constexpr double lower(double v, double e) { return (1.0 - e) * v; }
    static constexpr double upper(double v, double e) { return (1.0 + e) * v; }

    constexpr double yaw_vel_lower() const { return lower(yaw_vel_, yaw_error_); }

    // Return the first pose in the plan > lookahead distance away, or the last pose in the plan
    bool transform_pose(const geometry_msgs::msg::PoseStamped &in_pose,
                        geometry_msgs::msg::PoseStamped &out_pose,
                        const std::string &target_frame) const
    {
      try
      {
        auto transform = tf_->lookupTransform(
            target_frame, in_pose.header.frame_id,
            in_pose.header.stamp, tf2::durationFromSec(transform_tolerance_));
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_DEBUG(logger_, "Transform failed: %s", ex.what());
        return false;
      }
    }

    // Modified pure pursuit path tracking algorithm: works in 3D and supports deceleration
    // Reference "Implementation of the Pure Pursuit Path Tracking Algorithm" by R. Craig Coulter
    geometry_msgs::msg::Twist
    sharp_turn_3d(const geometry_msgs::msg::PoseStamped &pose_f_odom) const
    {
      // Transform pose odom -> map
      geometry_msgs::msg::PoseStamped pose_f_map;
      if (!transform_pose(pose_f_odom, pose_f_map, plan_.header.frame_id))
      {
        return geometry_msgs::msg::Twist{};
      }

      // Find goal
      auto goal_f_map = find_pure_pursuit_goal(plan_, pose_f_map, lookahead_dist_);

      // Plan poses are stale, update the timestamp to get recent map -> odom -> base transforms
      goal_f_map.header.stamp = pose_f_map.header.stamp;

      // Transform goal map -> base
      geometry_msgs::msg::PoseStamped goal_f_base;
      if (!transform_pose(goal_f_map, goal_f_base, base_frame_id_))
      {
        return geometry_msgs::msg::Twist{};
      }

      auto xy_dist_sq = goal_f_base.pose.position.x * goal_f_base.pose.position.x +
                        goal_f_base.pose.position.y * goal_f_base.pose.position.y;
      auto xy_dist_L2_norm = std::sqrt(xy_dist_sq);
      auto z_dist = std::abs(goal_f_base.pose.position.z);

      // Declaring the cmd_vel
      geometry_msgs::msg::Twist cmd_vel;

      // Calc linear.z
      if (z_dist > goal_tolerance_)
      {
        cmd_vel.linear.z = goal_f_base.pose.position.z > 0 ? z_vel_ : -z_vel_;

        // Decelerate
        z_limiter_.decelerate(cmd_vel.linear.z, goal_f_base.pose.position.z);
      }

      // Linear x velocity
      cmd_vel.linear.x = 0.0;

      // Calc angular.z ENU frame 
      if (xy_dist_L2_norm > goal_tolerance_)
      { 
        // Proportional Controller (self made)   
        cmd_vel.angular.z = 0;  // TODO: Formula here.
      } else
      {
        // Goal is behind the sub: rotate to face it
        cmd_vel.angular.z = goal_f_base.pose.position.y > 0 ? yaw_vel_ : -yaw_vel_;
      }

      return cmd_vel;
    }      


  public:
    SharpTurnController3D() = default;
    ~SharpTurnController3D() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &weak_parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
      auto parent = weak_parent.lock();

      logger_ = parent->get_logger();
      tf_ = tf;
      base_frame_id_ = costmap_ros->getBaseFrameID();
      

      PARAMETER(parent, name, z_vel, 0.2)
      PARAMETER(parent, name, z_accel, 0.2)
      PARAMETER(parent, name, yaw_vel, 0.4)
      PARAMETER(parent, name, yaw_accel, 0.4)
      PARAMETER(parent, name, lookahead_dist, 1.0)
      PARAMETER(parent, name, transform_tolerance, 1.0)
      PARAMETER(parent, name, goal_tolerance, 0.1)
      PARAMETER(parent, name, tick_rate, 20.0)
      PARAMETER(parent, name, publish_tracking_error, true)

      if (publish_tracking_error_) {
        cross_track_xy_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_cross_track_xy", rclcpp::QoS(10));
        vertical_error_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_vertical_error", rclcpp::QoS(10));
        yaw_error_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_yaw_error", rclcpp::QoS(10));
        closest_point_pub_ = parent->create_publisher<geometry_msgs::msg::PointStamped>(
          "pure_pursuit_closest_point_map", rclcpp::QoS(10));
        robot_pose_map_pub_ = parent->create_publisher<geometry_msgs::msg::PoseStamped>(
          "pure_pursuit_robot_pose_map", rclcpp::QoS(10));
        robot_twist_pub_ = parent->create_publisher<geometry_msgs::msg::TwistStamped>(
          "pure_pursuit_robot_twist", rclcpp::QoS(10));
      }

      z_limiter_ = Limiter(z_accel_, 1. / tick_rate_);
      yaw_limiter_ = Limiter(yaw_accel_, 1. / tick_rate_);

      transform_tolerance_d_ = rclcpp::Duration::from_seconds(transform_tolerance_);

      RCLCPP_INFO(logger_, "SharpTurnController3D configured");
    }

    void cleanup() override
    {
      cross_track_xy_pub_.reset();
      vertical_error_pub_.reset();
      yaw_error_pub_.reset();
      closest_point_pub_.reset();
      robot_pose_map_pub_.reset();
      robot_twist_pub_.reset();
    }

    void activate() override
    {
      if (cross_track_xy_pub_) {
        cross_track_xy_pub_->on_activate();
      }
      if (vertical_error_pub_) {
        vertical_error_pub_->on_activate();
      }
      if (yaw_error_pub_) {
        yaw_error_pub_->on_activate();
      }
      if (closest_point_pub_) {
        closest_point_pub_->on_activate();
      }
      if (robot_pose_map_pub_) {
        robot_pose_map_pub_->on_activate();
      }
      if (robot_twist_pub_) {
        robot_twist_pub_->on_activate();
      }
    }

    void deactivate() override
    {
      if (cross_track_xy_pub_) {
        cross_track_xy_pub_->on_deactivate();
      }
      if (vertical_error_pub_) {
        vertical_error_pub_->on_deactivate();
      }
      if (yaw_error_pub_) {
        yaw_error_pub_->on_deactivate();
      }
      if (closest_point_pub_) {
        closest_point_pub_->on_deactivate();
      }
      if (robot_pose_map_pub_) {
        robot_pose_map_pub_->on_deactivate();
      }
      if (robot_twist_pub_) {
        robot_twist_pub_->on_deactivate();
      }
    }

    // Pose is base_f_odom, it's 3D, and it comes from /tf via:
    //   nav2_controller::ControllerServer::getRobotPose
    //   Using the local_costmap (odom frame), not the global_costmap (map frame)
    //   nav2_costmap_2d::Costmap2DROS::getRobotPose
    //   nav2_util::getCurrentPose
    //
    // Twist comes from /odom, but it's stripped to 2D in nav2_controller::ControllerServer, so
    // ignore it.
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *) override
    {
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header = pose.header;

      if (publish_tracking_error_ && cross_track_xy_pub_ && vertical_error_pub_ && yaw_error_pub_ &&
        closest_point_pub_ && robot_pose_map_pub_ && robot_twist_pub_ && !plan_.poses.empty())
      {
        geometry_msgs::msg::PoseStamped pose_f_map;
        if (transform_pose(pose, pose_f_map, plan_.header.frame_id)) {
          double cross_xy = 0.0;
          double z_err = 0.0;
          double yaw_err = 0.0;
          geometry_msgs::msg::Point closest_map;
          tracking_errors_along_path(plan_, pose_f_map, cross_xy, z_err, yaw_err, closest_map);
          std_msgs::msg::Float64 cross_msg;
          cross_msg.data = cross_xy;
          cross_track_xy_pub_->publish(cross_msg);
          std_msgs::msg::Float64 z_msg;
          z_msg.data = z_err;
          vertical_error_pub_->publish(z_msg);
          std_msgs::msg::Float64 yaw_msg;
          yaw_msg.data = yaw_err;
          yaw_error_pub_->publish(yaw_msg);

          geometry_msgs::msg::PointStamped cp;
          cp.header.stamp = pose.header.stamp;
          cp.header.frame_id = plan_.header.frame_id;
          cp.point = closest_map;
          closest_point_pub_->publish(cp);

          pose_f_map.header.stamp = pose.header.stamp;
          robot_pose_map_pub_->publish(pose_f_map);

          geometry_msgs::msg::TwistStamped tw;
          tw.header.stamp = pose.header.stamp;
          tw.header.frame_id = pose.header.frame_id;
          tw.twist = velocity;
          robot_twist_pub_->publish(tw);
        }
      }

      // Track the plan
      cmd_vel.twist = sharp_turn_3d(pose);

      // Apply the limiters so we don't violate acceleration bounds!
      z_limiter_.limit(cmd_vel.twist.linear.z, prev_vel_.linear.z);
      yaw_limiter_.limit(cmd_vel.twist.angular.z, prev_vel_.angular.z);

      // Twist parameter from nav2_controller is generated from a Twist2D, so linear.z is always 0
      // Keep a copy of the previous cmd_vel instead
      prev_vel_ = cmd_vel.twist;

      return cmd_vel;
    }

    void setPlan(const nav_msgs::msg::Path &plan) override
    {
      if (plan.poses.empty())
      {
        throw nav2_core::PlannerException("Received plan with zero length");
      }
      plan_ = plan;
    }

    void setSpeedLimit(const double &, const bool &) override
    {
      // Not supported
      std::cout << "ERROR: speed limit is not supported" << std::endl;
    }
  };

} // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(orca_nav2::SharpTurnController3D, nav2_core::Controller)
