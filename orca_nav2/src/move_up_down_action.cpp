#include <algorithm>
#include <string>
#include <memory>
#include <stdexcept>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"


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
      BT::InputPort<double>("elapsed_time", 0.0,
        "Descent duration [s]. "
        "If > 0: move for this duration then return SUCCESS. "
        "If 0/unset: ascending mode — move indefinitely; writes ascent duration "
        "to blackboard key 'ascend_duration' on halt."),
    };
  }
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void send_velocity(double z_vel);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;

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
  cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("/pixhawk/cmd_vel", 10);
}

void MoveUpDownAction::send_velocity(double z_vel)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.z = z_vel;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  cmd_vel_->publish(msg);
}

BT::NodeStatus MoveUpDownAction::tick()
{
  rclcpp::spin_some(node_);

  double speed = 0.0;
  if (!getInput<double>("speed", speed)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveUpDownAction"), "Failed to get 'speed' input");
    return BT::NodeStatus::FAILURE;
  }

  double descent_duration = 0.0;
  getInput<double>("elapsed_time", descent_duration);

  if (!timing_) {
    start_time_ = node_->now();
    timing_ = true;
  }

  send_velocity(speed);

  // Descent mode: count down and self-terminate
  if (descent_duration > 1e-3) {
    const double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed >= descent_duration) {
      send_velocity(0.0);
      timing_ = false;
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Ascending mode (or descent still running): keep going
  return BT::NodeStatus::RUNNING;
}

void MoveUpDownAction::halt()
{
  send_velocity(0.0);

  if (timing_) {
    const double elapsed = std::max(
      (node_->now() - start_time_).seconds(),
      1e-3);

    // Only store ascent duration when halted from ascending mode (no elapsed_time set)
    double descent_duration = 0.0;
    getInput<double>("elapsed_time", descent_duration);
    if (descent_duration <= 1e-3) {
      config().blackboard->set<double>("ascend_duration", elapsed);
      RCLCPP_INFO(
        rclcpp::get_logger("MoveUpDownAction"),
        "Stored ascent duration: %.2f s", elapsed);
    }

    timing_ = false;
  }

  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::MoveUpDownAction>("MoveUpDownAction");
}
