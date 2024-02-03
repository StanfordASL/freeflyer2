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


#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "ff_drivers/gpio.hpp"
#include "ff_msgs/msg/thruster_command.hpp"

#define NUM_THRUSTERS 8

// thruster pin connection
// @see: https://wiki.odroid.com/odroid-n2l/application_note/gpio/pwm#tab__odroid-n2
static constexpr int THRUSTER_PINS[NUM_THRUSTERS] = {
  476,  // thruster pin 1 -> odroid pin 16
  477,  // thruster pin 2 -> odroid pin 18
  484,  // thruster pin 3 -> odroid pin 19
  485,  // thruster pin 4 -> odroid pin 21
  478,  // thruster pin 5 -> odroid pin 22
  487,  // thruster pin 6 -> odroid pin 23
  486,  // thruster pin 7 -> odroid pin 24
  464,  // thruster pin 8 -> odroid pin 26
};


using ff_msgs::msg::ThrusterCommand;

class ThrusterNode : public rclcpp::Node
{
public:
  ThrusterNode()
  : rclcpp::Node("thruster_node")
  {
    // initialize all GPIO
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
      gpios_.push_back(std::make_unique<ff::GPIO>(THRUSTER_PINS[i]));
    }

    // listen to commands
    sub_thruster_ = this->create_subscription<ThrusterCommand>(
      "commands/binary_thrust",
      10,
      [this](const ThrusterCommand::SharedPtr msg) {ThrusterCommandCallback(msg);});
  }

private:
  std::vector<std::unique_ptr<ff::GPIO>> gpios_;
  rclcpp::Subscription<ThrusterCommand>::SharedPtr sub_thruster_;

  void ThrusterCommandCallback(const ThrusterCommand::SharedPtr msg)
  {
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
      if (gpios_[i]->GetState() != msg->switches[i]) {
        gpios_[i]->SetState(msg->switches[i]);
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterNode>());
  rclcpp::shutdown();
  return 0;
}
