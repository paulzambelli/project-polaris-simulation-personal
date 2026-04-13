#include <string>
#include <memory>
#include <stdexcept>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace orca_nav2
{

class GoUpToIceAction : public BT::ActionNodeBase
{
public:
  GoUpToIceAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("upward_speed", 0.05, "Upward speed for ice measurement."),
    };
  }
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void send_velocity(double z_vel);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_;
};


GoUpToIceAction::GoUpToIceAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("GoUpToIceAction"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }
  cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/pixhawk/cmd_vel", 10);
}

void GoUpToIceAction::send_velocity(double z_vel)
{
  geometry_msgs::msg::TwistStamped msg;
  
  // Fill the Header
  msg.header.stamp = node_->now();
  msg.header.frame_id = "base_link";

  // Fill the Twist
  msg.twist.linear.z = z_vel;
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
  
  cmd_vel_->publish(msg);
}

BT::NodeStatus GoUpToIceAction::tick()
{
  double upward_speed;
  if (!getInput<double>("upward_speed", upward_speed)) {
    RCLCPP_ERROR(rclcpp::get_logger("GoUpToIceAction"), "Failed to get 'upward_speed' input");
    return BT::NodeStatus::FAILURE;
  }

  send_velocity(upward_speed);

  return BT::NodeStatus::RUNNING;
}

void GoUpToIceAction::halt()
{
  // THIS IS VITAL: Stop the robot when the action is interrupted by the Condition node!
  send_velocity(0.0);

  // You must call the base class halt() at the end
  BT::ActionNodeBase::halt();
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::GoUpToIceAction>("GoUpToIceAction");
}

