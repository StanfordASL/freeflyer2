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


#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_drivers/pwm.hpp"
#include "ff_msgs/msg/thruster_command.hpp"

#define NUM_THRUSTERS 8

// thruster pin connection
// @see: https://wiki.odroid.com/odroid-n2l/application_note/gpio/pwm#tab__odroid-n2
static constexpr int THRUSTER_PINS[NUM_THRUSTERS] = {
  473,  // thruster pin 1 -> odroid pin 7
  479,  // thruster pin 2 -> odroid pin 11
  480,  // thruster pin 3 -> odroid pin 13
  472,  // thruster pin 4 -> odroid pin 32
  495,  // thruster pin 5 -> odroid pin 36
  476,  // thruster pin 6 -> odroid pin 16
  477,  // thruster pin 7 -> odroid pin 18
  478,  // thruster pin 8 -> odroid pin 22
};


using namespace std::chrono_literals;
using ff_msgs::msg::ThrusterCommand;

class ThrusterNode : public ff::PWMManager
{
public:
  ThrusterNode()
  : ff::PWMManager("thruster_driver_node")
  {
    // add all PWMs
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
      this->AddSoftPWM(THRUSTER_PINS[i]);
    }

    // set period (default to 10Hz)
    double period = this->declare_parameter("period", .1);
    this->SetPeriodAll(period * 1s);
    // update period on the fly
    sub_params_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_period_ = sub_params_->add_parameter_callback(
      "period",
      [this](const rclcpp::Parameter & p) {SetPeriodAll(p.as_double() * 1s);});

    // start all PWMs
    this->EnableAll();

    // listen to commands
    sub_duty_cycle_ = this->create_subscription<ThrusterCommand>(
      "commands/duty_cycle",
      10, [this](const ThrusterCommand::SharedPtr msg) {DutyCycleCallback(msg);});
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> sub_params_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_period_;
  rclcpp::Subscription<ThrusterCommand>::SharedPtr sub_duty_cycle_;

  void DutyCycleCallback(const ThrusterCommand::SharedPtr msg)
  {
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
      this->SetDutyCycle(i, msg->duty_cycle[i]);
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
