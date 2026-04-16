#include <cmath>
#include <string>
#include <memory>
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"


namespace orca_nav2
{

class IsLongEnoughDown : public BT::ConditionNode
{
public:
  IsLongEnoughDown(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_time", "Minimum time to go down [s]."),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  bool timing_ = false;
};


IsLongEnoughDown::IsLongEnoughDown(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
    if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("IsLongEnoughDown"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }
}

BT::NodeStatus IsLongEnoughDown::tick()
{
  // Same Nav2 BT client-node story as IsCloseToIce: pump so sim /clock advances for node_->now().
  rclcpp::spin_some(node_);

  double min_time = 0.0;
  // ReactiveFallback ticks this node before MoveUpDown. If min_time is 0 (unset ascend_duration or
  // ascent halt wrote 0 s), the first tick would succeed immediately and the descend action never runs.
  constexpr double kMinValid = 1e-3;
  constexpr double kDefaultSeconds = 5.0;
  if (!getInput("min_time", min_time) || min_time < kMinValid) {
    if (!config().blackboard->get<double>("ascend_duration", min_time) || min_time < kMinValid) {
      min_time = kDefaultSeconds;
    }
  }

  if (!timing_) {
    start_time_ = node_->now();
    timing_ = true;
  }

  double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= min_time) {
    timing_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsLongEnoughDown>("IsLongEnoughDown");
}

