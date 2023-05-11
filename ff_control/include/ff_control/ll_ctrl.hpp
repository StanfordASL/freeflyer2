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
  /**
   * @brief robot parameters that can be accessed by sub-classes
   */
  const ff::RobotParams p_;

  /**
   * @brief send command to set the thrusters duty cycles
   *
   * @param duty_cycle duty cycle for each thruster (in [0, 1])
   */
  void SetThrustDutyCycle(const std::array<double, 8>& duty_cycle);

  /**
   * @brief send command to set the inertial wheel velocity
   *        @TODO(alvin): support this or remove?
   *
   * @param velocity velocity in [m/s]
   */
  void SetWheelVelocity(const double& velocity);

 private:
  rclcpp::Publisher<ff_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
  rclcpp::Publisher<ff_msgs::msg::WheelVelCommand>::SharedPtr wheel_pub_;
};

} // namespace ff
