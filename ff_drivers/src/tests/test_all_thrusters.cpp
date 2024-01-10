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

#include "ff_control/pwm_ctrl.hpp"

using namespace std::chrono_literals;


class TestAllThrustersNode : public ff::PWMController
{
public:
  TestAllThrustersNode()
  : rclcpp::Node("test_all_thrusters_node"),
    ff::PWMController()
  {
    timer_ = this->create_wall_timer(5s, std::bind(&TestAllThrustersNode::TimerCallback, this));
    this->declare_parameter("duty_cycle", .2);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int th_idx_ = 0;

  void TimerCallback()
  {
    double duty_cycle = this->get_parameter("duty_cycle").as_double();

    std::array<double, 8> duty_cycles;
    // populate thrust msg
    for (int i = 0; i < 8; ++i) {
      duty_cycles[i] = 0.;
      if (i == th_idx_) {
        duty_cycles[i] = duty_cycle;
      }
    }

    // publish thrust msg
    SetThrustDutyCycle(duty_cycles);
    RCLCPP_INFO(this->get_logger(), "opening valve %d", th_idx_);

    // increment th_idx
    th_idx_ = (th_idx_ + 1) % 8;
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestAllThrustersNode>());
  rclcpp::shutdown();
  return 0;
}
