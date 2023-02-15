#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ff_params/robot_params.hpp"

namespace ff {

class LowLevelController : public rclcpp::Node {
 public:
  explicit LowLevelController(const std::string& node_name);

 protected:
  const ff::RobotParams p_;

  void SetThrustDutyCycle(const std_msgs::msg::Float64MultiArray& msg);
  void SetWheelVelocity(const std_msgs::msg::Float64& msg);

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_pub_;
};

} // namespace ff
