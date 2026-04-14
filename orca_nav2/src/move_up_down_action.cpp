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

class MoveUpDownAction : public BT::ActionNodeBase
{
public:
  MoveUpDownAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("speed", 0.0, "Vertical speed in base_link (+z up, -z down) [m/s]."),
      BT::OutputPort<double>("elapsed_time", "Time it takes to move up/down [s]."),
    };
  }
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void send_velocity(double z_vel);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_;

  rclcpp::Time start_time_;
  bool timing_ = false;
};


MoveUpDownAction::MoveUpDownAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveUpDownAction"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }
  cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/pixhawk/cmd_vel", 10);
}

void MoveUpDownAction::send_velocity(double z_vel)
{
  geometry_msgs::msg::TwistStamped msg;

  msg.header.stamp = node_->now();
  msg.header.frame_id = "base_link";

  msg.twist.linear.z = z_vel;
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;

  cmd_vel_->publish(msg);
}

BT::NodeStatus MoveUpDownAction::tick()
{
  double speed = 0.0;
  if (!getInput<double>("speed", speed)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveUpDownAction"), "Failed to get 'speed' input");
    return BT::NodeStatus::FAILURE;
  }

  if (!timing_) {
    start_time_ = node_->now();
    timing_ = true;
  }

  send_velocity(speed);

  return BT::NodeStatus::RUNNING;
}

void MoveUpDownAction::halt()
{
  send_velocity(0.0);  

  if (timing_) {
    double elapsed = (node_->now() - start_time_).seconds();
     auto res = setOutput("elapsed_time", elapsed);
    if (res) {
      RCLCPP_INFO(rclcpp::get_logger("MoveUpDownAction"), "Stored elapsed time: %.2f seconds", elapsed);
    }
    timing_ = false;
  }

  BT::ActionNodeBase::halt();
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::MoveUpDownAction>("MoveUpDownAction");
}
