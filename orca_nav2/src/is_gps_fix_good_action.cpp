#include <cmath>
#include <string>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <functional>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

namespace orca_nav2
{

// ActionNodeBase (not ConditionNode) so we can return RUNNING while waiting for a good fix.
// Behaviour:
//   RUNNING  — no message received yet, or covariance type unknown, or accuracy still too poor.
//   SUCCESS  — message received and horizontal accuracy <= max_xy_accuracy.
//   FAILURE  — hard error only (bad port input).
//
// This cooperates correctly with a Timeout parent: the node keeps returning RUNNING until
// GPS is good; if the Timeout fires it halts the node and the Fallback's AlwaysSuccess takes over.
// No RetryUntilSuccessful wrapper is needed in the tree.
class IsGPSFixGood : public BT::ActionNodeBase
{
public:
  IsGPSFixGood(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_xy_accuracy", 2.0, "Maximum allowed horizontal accuracy [m]."),
    };
  }

  BT::NodeStatus tick() override;

  // Subscription lives for the lifetime of the node — nothing to clean up on halt.
  void halt() override { setStatus(BT::NodeStatus::IDLE); }

private:
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  std::atomic<double> current_xy_acc_{-1.0};
  std::atomic<bool> has_received_data_{false};
};


IsGPSFixGood::IsGPSFixGood(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("IsGPSFixGood"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }

  subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix",
    rclcpp::SensorDataQoS(),
    std::bind(&IsGPSFixGood::gpsFixCallback, this, _1)
  );
}

void IsGPSFixGood::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  double xy_acc = -1.0;
  const auto t = msg->position_covariance_type;
  const auto & cov = msg->position_covariance;

  if (t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
    if (cov.size() >= 5) {
      xy_acc = std::sqrt(std::max(0.0, std::max(cov[0], cov[4])));
    }
  } else if (
    t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN ||
    t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED)
  {
    if (cov.size() >= 9) {
      double a = cov[0];
      double b = cov[1];
      double d = cov[4];
      double trace = a + d;
      double det = a * d - b * b;
      double disc = std::max(0.0, trace * trace - 4.0 * det);
      double lam_max = 0.5 * (trace + std::sqrt(disc));
      xy_acc = std::sqrt(std::max(0.0, lam_max));
    }
  }
  // COVARIANCE_TYPE_UNKNOWN → xy_acc stays -1.0 (fix not ready)

  current_xy_acc_.store(xy_acc);
  has_received_data_.store(true);
}


BT::NodeStatus IsGPSFixGood::tick()
{
  rclcpp::spin_some(node_);

  double max_xy_accuracy;
  if (!getInput("max_xy_accuracy", max_xy_accuracy)) {
    RCLCPP_ERROR(rclcpp::get_logger("IsGPSFixGood"), "Failed to get 'max_xy_accuracy' input");
    return BT::NodeStatus::FAILURE;
  }

  if (!has_received_data_.load()) {
    // No /fix message yet — publisher may not be up, keep waiting.
    return BT::NodeStatus::RUNNING;
  }

  double xy_acc = current_xy_acc_.load();

  if (xy_acc < 0.0) {
    // Message received but covariance type unknown — satellite search still in progress.
    return BT::NodeStatus::RUNNING;
  }

  if (xy_acc <= max_xy_accuracy) {
    RCLCPP_INFO(
      rclcpp::get_logger("IsGPSFixGood"),
      "GPS fix good: xy_acc=%.2f m (threshold=%.2f m)", xy_acc, max_xy_accuracy);
    return BT::NodeStatus::SUCCESS;
  }

  // Accuracy too poor — keep waiting for a better fix.
  RCLCPP_DEBUG(
    rclcpp::get_logger("IsGPSFixGood"),
    "GPS fix poor: xy_acc=%.2f m (threshold=%.2f m)", xy_acc, max_xy_accuracy);
  return BT::NodeStatus::RUNNING;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsGPSFixGood>("IsGPSFixGood");
}
