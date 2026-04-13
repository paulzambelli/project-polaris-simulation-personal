#include <string>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <functional>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

namespace orca_nav2
{

class IsGPSFixGood : public BT::ConditionNode
{
public:
  IsGPSFixGood(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_xy_accuracy", 2.0, "Maximum allowed horizontal accuracy in meters"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  std::atomic<double> current_xy_acc_{-1.0};
  std::atomic<bool> has_received_data_{false};
};


IsGPSFixGood::IsGPSFixGood(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  rclcpp::Node::SharedPtr node;

  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node)) {
    RCLCPP_ERROR(rclcpp::get_logger("IsGPSFixGood"), "Failed to get 'node' from blackboard");
    throw std::runtime_error("Failed to get 'node' from blackboard");
  }
  
  subscription_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix",
    rclcpp::SensorDataQoS(),
    std::bind(&IsGPSFixGood::gpsFixCallback, this, _1)
  );
}

void IsGPSFixGood::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  double xy_acc = -1.0;
  auto t = msg->position_covariance_type;
  const auto& cov = msg->position_covariance;

  if (t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    xy_acc = -1.0;
  } 
  else if (t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
    if (cov.size() >= 5) {
      xy_acc = std::sqrt(std::max(0.0, std::max(cov[0], cov[4])));
    }
  } 
  else if (t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN ||
             t == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
    if (cov.size() >= 9) {
      // Calculate largest horizontal 1-sigma (sqrt of max eigenvalue of East-North 2x2 block)
      double a = cov[0];
      double b = cov[1];
      double d = cov[4];
      double trace = a + d;
      double det = a * d - b * b;
      double disc = trace * trace - 4.0 * det;
      if (disc < 0.0) {
        disc = 0.0;
      }
      double lam_max = 0.5 * (trace + std::sqrt(disc));
      xy_acc = std::sqrt(std::max(0.0, lam_max));
    }
  }

  current_xy_acc_.store(xy_acc);
  has_received_data_.store(true);
}


BT::NodeStatus IsGPSFixGood::tick()
{
  double max_xy_accuracy;
  
  if (!has_received_data_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("max_xy_accuracy", max_xy_accuracy)) {
    return BT::NodeStatus::FAILURE;
  }

  double xy_acc = current_xy_acc_.load();

  if (xy_acc < 0.0) {
    return BT::NodeStatus::FAILURE;
  }

  // Check if our calculated accuracy meets the strict threshold
  if (xy_acc <= max_xy_accuracy) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsGPSFixGood>("IsGPSFixGood");
}

