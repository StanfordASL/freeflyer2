#include <fcntl.h>
#include <termio.h>
#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/wrench2_d.hpp"
#include "ff_control/wrench_ctrl.hpp"

using namespace std::chrono_literals;

class KeyboardTeleopNode : public ff::WrenchController {
 public:
  KeyboardTeleopNode() : ff::WrenchController("key_teleop_node") {
    // setup non blocking termio
    struct termios new_settings;
    tcgetattr(STDIN_FILENO, &old_settings_);
    tcgetattr(STDIN_FILENO, &new_settings);
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    f_flags_ = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, f_flags_ | O_NONBLOCK);

    last_key_time_ = this->get_clock()->now();

    // start teleop loop
    telop_timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardTeleopNode::TeleopLoop, this));
    key_timer_ = this->create_wall_timer(5ms, std::bind(&KeyboardTeleopNode::KeyChecker, this));
  }

  ~KeyboardTeleopNode() {
    // restore termio settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
    fcntl(STDIN_FILENO, F_SETFL, f_flags_);
  }

 private:
  struct termios old_settings_;
  int f_flags_;

  char last_key_ = 0;
  rclcpp::Time last_key_time_;

  rclcpp::TimerBase::SharedPtr telop_timer_;
  rclcpp::TimerBase::SharedPtr key_timer_;

  void KeyChecker() {
    const char key = getchar();
    if (key < 0) {
      if (this->get_clock()->now() - last_key_time_ > 1s) {
        last_key_ = 0;
      }
    } else {
      last_key_ = key;
      last_key_time_ = this->get_clock()->now();
    }
  }

  void TeleopLoop() {
    ff_msgs::msg::Wrench2D wrench_body{};

    switch (last_key_) {
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
