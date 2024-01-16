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
   * @brief send binary switching command to the thrusters
   *
   * @param thrusts boolean switches for each thruster (True is on, False is off)
   */
  void SetAllThrusts(const std::array<bool, 8> & thrusts);

  /**
   * @brief set single thrust command
   *
   * @param idx index of the target thruster
   * @param on  set to true to turn on thruster, false to turn off
   */
  void SetThrust(size_t idx, bool on);

  /**
   * @brief obtain the current state of thrusters
   *
   * @return boolean states for each thruster
   */
  const std::array<bool, 8> & GetThrust() const;

  /**
   * @brief send command to set the inertial wheel velocity
   *        @TODO(alvin): support this or remove?
   *
   * @param velocity angular velocity in [rad/s]
   */
  void SetWheelVelocity(const double & velocity);

private:
  rclcpp::Publisher<ff_msgs::msg::ThrusterCommand>::SharedPtr thrust_pub_;
  rclcpp::Publisher<ff_msgs::msg::WheelVelCommand>::SharedPtr wheel_pub_;
  ff_msgs::msg::ThrusterCommand thrust_cmd_{};
};

}  // namespace ff
