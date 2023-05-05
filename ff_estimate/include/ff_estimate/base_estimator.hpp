#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

namespace ff {

class BaseEstimator : public rclcpp::Node {
 public:
  BaseEstimator(const std::string& node_name);

 protected:
  void SendStateEstimate(const ff_msgs::msg::FreeFlyerState& state);

 private:
  rclcpp::Publisher<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_pub_;
};

} // namespace ff
