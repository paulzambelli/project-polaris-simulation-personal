#ifndef ORCA_NAV2__SHARP_TURN_CONTROLLER3D_HPP_
#define ORCA_NAV2__SHARP_TURN_CONTROLLER3D_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <cassert>

#include "orca_nav2/param_macro.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
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

class Limiter
{
    double max_a_{};
    double max_dv_{};

  public:
    Limiter() = default;
    ~Limiter() = default;

    Limiter(double max_accel, double dt);
    
    void decelerate(double &v, const double &goal_dist);
    void limit(double &v, const double &prev_v);
};

class SharpTurnController3D : public nav2_core::Controller
{
    public:

        SharpTurnController3D() = default;
        ~SharpTurnController3D() override = default;
        
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        void setPlan(
            const nav_msgs::msg::Path & path
        ) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * goal_checker
        ) override;

    private:
        double yaw_vel_;
        double yaw_p_gain_;

        nav_msgs::msg::Path global_plan_;
};

} // namespace orca_nav2

#endif  // ORCA_NAV2__SHARP_TURN_CONTROLLER3D_HPP_