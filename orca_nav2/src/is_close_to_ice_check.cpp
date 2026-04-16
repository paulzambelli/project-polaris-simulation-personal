#include <cmath>
#include <string>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <functional>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

namespace orca_nav2
{

class IsCloseToIce : public BT::ConditionNode
{
public:
  IsCloseToIce(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_dist_to_ice", 0.3, "Maximum distance from beneath the ice to stop going up [m]."),
    };
  }

  BT::NodeStatus tick() override;

private:
  void distanceToIceCallback(std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  std::atomic<double> current_ice_distance_{0.0};
  std::atomic<bool> has_received_data_{false};
};


IsCloseToIce::IsCloseToIce(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("IsCloseToIce"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }

  subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/top/ultrasonic/distance",
    rclcpp::SensorDataQoS(),
    std::bind(&IsCloseToIce::distanceToIceCallback, this, _1)
  );
}

void IsCloseToIce::distanceToIceCallback(std_msgs::msg::Float32::SharedPtr msg)
{
  current_ice_distance_.store(msg->data);
  has_received_data_.store(true);
}


BT::NodeStatus IsCloseToIce::tick()
{
  // Blackboard "node" is Nav2's internal BT client node; nothing spins it globally, so
  // subscriptions only run if we pump the executor here (same pattern as Nav2's
  // GoalCheckerSelector / IsBatteryLowCondition).
  rclcpp::spin_some(node_);

  double max_dist_to_ice;

  if (!getInput("max_dist_to_ice", max_dist_to_ice)) {
    return BT::NodeStatus::FAILURE;
  }
  // Without range data we must not pretend "at ice": that skips the entire ascent branch
  // (ReactiveFallback succeeds on first child) and leaves ascend_duration unset.
  if (!has_received_data_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  if (std::abs(current_ice_distance_.load()) > max_dist_to_ice) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsCloseToIce>("IsCloseToIce");
}

