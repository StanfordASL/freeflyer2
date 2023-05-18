// MIT License
//
// Copyright (c) 2023 Stanford Autonomous Systems Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/thruster_command.hpp"
#include "ff_msgs/msg/wheel_vel_command.hpp"
#include "ff_params/robot_params.hpp"

namespace ff
{

class LowLevelController : virtual public rclcpp::Node
{
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
  void SetThrustDutyCycle(const std::array<double, 8> & duty_cycle);

  /**
   * @brief send command to set the inertial wheel velocity
   *        @TODO(alvin): support this or remove?
   *
   * @param velocity velocity in [m/s]
   */
  void SetWheelVelocity(const double & velocity);

private:
  rclcpp::Publisher<ff_msgs::msg::ThrusterCommand>::SharedPtr thruster_pub_;
  rclcpp::Publisher<ff_msgs::msg::WheelVelCommand>::SharedPtr wheel_pub_;
};

}  // namespace ff
