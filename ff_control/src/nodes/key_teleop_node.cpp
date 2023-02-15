#include <fcntl.h>
#include <termio.h>
#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/wrench2_d.hpp"
#include "ff_control/wrench_ctrl.hpp"
#include "ff_control/keyboard_ctrl.hpp"

using namespace std::chrono_literals;

class KeyboardTeleopNode : public ff::WrenchController, public ff::KeyboardController {
 public:
  KeyboardTeleopNode()
    : rclcpp::Node("key_teleop_node"),
      ff::WrenchController(),
      ff::KeyboardController() {
    // start teleop loop
    telop_timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardTeleopNode::TeleopLoop, this));
  }

 private:
  rclcpp::TimerBase::SharedPtr telop_timer_;

  void TeleopLoop() {
    ff_msgs::msg::Wrench2D wrench_body{};

    switch (this->GetKey()) {
    case 'w':
      wrench_body.fx = p_.actuators.F_body_max / 2;
      break;
    case 's':
      wrench_body.fx = -p_.actuators.F_body_max / 2;
      break;
    case 'a':
      wrench_body.fy = p_.actuators.F_body_max / 2;
      break;
    case 'd':
      wrench_body.fy = -p_.actuators.F_body_max / 2;
      break;
    case 'q':
      wrench_body.tz = p_.actuators.M_body_max / 2;
      break;
    case 'e':
      wrench_body.tz = -p_.actuators.M_body_max / 2;
      break;
    }

    SetBodyWrench(wrench_body);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
}
