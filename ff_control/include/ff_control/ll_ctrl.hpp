#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/thruster_command.hpp"
#include "ff_msgs/msg/wheel_vel_command.hpp"
#include "ff_params/robot_params.hpp"

namespace ff {

class LowLevelController : virtual public rclcpp::Node {
 public:
  LowLevelController();

 protected:
  const ff::RobotParams p_;

  void SetThrustDutyCycle(const std::array<double, 8>& duty_cycle);
  void SetWheelVelocity(const double& velocity);

 private:
  rclcpp::Publisher<ff_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
  rclcpp::Publisher<ff_msgs::msg::WheelVelCommand>::SharedPtr wheel_pub_;
};

} // namespace ff
