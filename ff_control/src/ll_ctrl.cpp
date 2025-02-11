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


#include "ff_control/ll_ctrl.hpp"

using ff_msgs::msg::ThrusterCommand;
using ff_msgs::msg::WheelVelCommand;

namespace ff
{

LowLevelController::LowLevelController()
: rclcpp::Node("ll_ctrl_node"),
  p_(this)
{
  thrust_pub_ = this->create_publisher<ThrusterCommand>("ctrl/binary_thrust", 10);
  wheel_pub_ = this->create_publisher<WheelVelCommand>("ctrl/velocity", 10);
}

void LowLevelController::SetAllThrusts(const std::array<bool, 8> & thrusts)
{
  thrust_cmd_.header.stamp = this->get_clock()->now();
  thrust_cmd_.switches = thrusts;

  thrust_pub_->publish(thrust_cmd_);
}

void LowLevelController::SetThrust(size_t idx, bool on)
{
  if (idx > thrust_cmd_.switches.size()) {
    RCLCPP_ERROR(this->get_logger(), "thrust index out of range");
    return;
  }

  thrust_cmd_.header.stamp = this->get_clock()->now();
  thrust_cmd_.switches[idx] = on;

  thrust_pub_->publish(thrust_cmd_);
}

const std::array<bool, 8> & LowLevelController::GetThrust() const
{
  return thrust_cmd_.switches;
}

void LowLevelController::SetWheelVelocity(const double & velocity)
{
  WheelVelCommand msg{};
  msg.header.stamp = this->get_clock()->now();
  msg.velocity = velocity;

  wheel_pub_->publish(msg);
}

}  // namespace ff
