// MIT License
//
// Copyright (c) 2024 Stanford Autonomous Systems Lab
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


#include <atomic>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_control/pwm_ctrl.hpp"
#include "ff_msgs/msg/thruster_pwm_command.hpp"

using namespace std::placeholders;
using ff_msgs::msg::ThrusterPWMCommand;

class PWMControlNode : public ff::PWMController
{
public:
  PWMControlNode()
  : rclcpp::Node("pwm_control_node"),
    ff::PWMController()
  {
    thruster_pwm_sub_ = this->create_subscription<ThrusterPWMCommand>(
      "ctrl/pwm_thrust", 10, std::bind(&PWMControlNode::PWMThrustCallback, this, _1));
  }

private:
  rclcpp::Subscription<ThrusterPWMCommand>::SharedPtr thruster_pwm_sub_;

  void PWMThrustCallback(const ThrusterPWMCommand::SharedPtr msg)
  {
    SetThrustDutyCycle(msg->duty_cycles);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PWMControlNode>());
  rclcpp::shutdown();
  return 0;
}
