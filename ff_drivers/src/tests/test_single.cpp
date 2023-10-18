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

#include <rclcpp/rclcpp.hpp>

#include "ff_drivers/pwm.hpp"

using namespace std::chrono_literals;


class TestSingleNode : public ff::PWMManager
{
public:
  TestSingleNode()
  : ff::PWMManager("test_single_node")
  {
    bool use_hard = this->declare_parameter("use_hard", true);
    int pin = this->declare_parameter("pin", 473);
    //this->declare_parameter("max_duty_cycle", .2);
    this->declare_parameter("curr_duty_cycle",0.2);

    if (use_hard) {
      RCLCPP_INFO(this->get_logger(), "Using hardware PWM: PWM_C");
      this->AddHardPWM(ff::HardPWM::PWM_C);
    } else {
      RCLCPP_INFO(this->get_logger(), "Using software PWM: pin %d", pin);
      this->AddSoftPWM(pin);
    }

    this->SetPeriodAll(10s);
    this->EnableAll();

    timer_ = this->create_wall_timer(1s, [this]() {TimerCallback();});
  }

private:
  void TimerCallback()
  {
    //double max_duty_cycle = this->get_parameter("max_duty_cycle").as_double();
    double new_duty_cycle = this->get_parameter("curr_duty_cycle").as_double();
    //this->SetDutyCycle(0, cnt_ * max_duty_cycle * .1);
    this->SetDutyCycle(0, new_duty_cycle);
    //RCLCPP_INFO(this->get_logger(), "Dutycycle set to %f", cnt_ * max_duty_cycle * .1);
    RCLCPP_INFO(this->get_logger(), "Dutycycle set to %f", new_duty_cycle);
    //cnt_ = (cnt_ + 1) % 11;
  }

  int cnt_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSingleNode>());
  rclcpp::shutdown();
  return 0;
}
