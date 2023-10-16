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


#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_control/linear_ctrl.hpp"
#include "ff_control/keyboard_ctrl.hpp"

using namespace std::chrono_literals;

const char help_msg[] =
  R"(
This keyboard teleop node controls the linear and angular velocity of the
freeflyer using a simple proportional controller. There will be large delay
in your control actions due to low control authority.

DRIVE WITH CAUTION and REACT EARLY!!!

Use the following keys to control the robot in global cooridnate frame
(lifting the key will eventually put the robot to a stop)

  Q  W  E
  A  S  D

W/S -- translate the robot in positive / negative x direction
A/D -- translate the robot in positive / negative y direction
Q/E -- rotate the robot (q counter clockwise, e clockwise)
)";

class KeyboardTeleopNode : public ff::LinearController, public ff::KeyboardController
{
public:
  KeyboardTeleopNode()
  : rclcpp::Node("key_teleop_node"),
    ff::LinearController(),
    ff::KeyboardController()
  {
    // start teleop loop
    telop_timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardTeleopNode::TeleopLoop, this));

    // build linear feedback control matrix
    K_.fill(0);
    const double gain_df = declare_parameter("gain_df", 10.0);
    const double gain_dt = declare_parameter("gain_dt", 0.4);
    K_(0, 3) = gain_df;
    K_(1, 4) = gain_df;
    K_(2, 5) = gain_dt;

    // print help message
    RCLCPP_INFO(this->get_logger(), "%s", help_msg);
  }

private:
  rclcpp::TimerBase::SharedPtr telop_timer_;

  FeedbackMat K_;

  void TeleopLoop()
  {
    if (!StateIsReady()) {return;}

    ff_msgs::msg::FreeFlyerState state_des{};
    switch (this->GetKey()) {
      case 'w':
        state_des.twist.vx = 0.3;
        break;
      case 's':
        state_des.twist.vx = -0.3;
        break;
      case 'a':
        state_des.twist.vy = 0.3;
        break;
      case 'd':
        state_des.twist.vy = -0.3;
        break;
      case 'q':
        state_des.twist.wz = 0.5;
        break;
      case 'e':
        state_des.twist.wz = -0.5;
        break;
    }

    SendControl(state_des, K_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
}