#include "ff_estimate/base_estimator.hpp"

using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::FreeFlyerStateStamped;

namespace ff {

BaseEstimator::BaseEstimator(const std::string& node_name) : rclcpp::Node(node_name) {
  state_pub_ = this->create_publisher<FreeFlyerStateStamped>("est/state", 10);
}

void BaseEstimator::SendStateEstimate(const FreeFlyerState& state) {
  ff_msgs::msg::FreeFlyerStateStamped msg{};
  msg.state = state;
  msg.header.stamp = this->get_clock()->now();

  state_pub_->publish(msg);
}

} // namespace ff
