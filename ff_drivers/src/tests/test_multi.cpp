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


class TestMultiNode : public ff::PWMManager
{
public:
  TestMultiNode()
  : ff::PWMManager("test_multi_node")
  {
    //this->declare_parameter("max_duty_cycle", .2);
    this->declare_parameter("curr_duty_cycle",0.1);

    this->AddSoftPWM(476);
    this->AddSoftPWM(477);
    this->AddSoftPWM(484);
    this->AddSoftPWM(485);
    this->AddSoftPWM(478);
    this->AddSoftPWM(487);
    this->AddSoftPWM(486);
    this->AddSoftPWM(464);

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
    for (int i = 0; i < 8; ++i) {
      this->SetDutyCycle(i, new_duty_cycle);
    }
    //RCLCPP_INFO(this->get_logger(), "Dutycycle set to %f", cnt_ * max_duty_cycle * .1);
    RCLCPP_INFO(this->get_logger(), "Dutycycle set to %f", new_duty_cycle);
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestMultiNode>());
  rclcpp::shutdown();
  return 0;
}
