#include <fcntl.h>
#include <chrono>

#include "ff_control/keyboard_ctrl.hpp"

using namespace std::chrono_literals;

namespace ff {

KeyboardController::KeyboardController()
  : rclcpp::Node("keyboard_ctrl_node") {
  // setup non blocking termio
  struct termios new_settings;
  tcgetattr(fileno(stdin), &old_settings_);
  tcgetattr(fileno(stdin), &new_settings);
  new_settings.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fileno(stdin), TCSANOW, &new_settings);

  f_flags_ = fcntl(fileno(stdin), F_GETFL);
  fcntl(fileno(stdin), F_SETFL, f_flags_ | O_NONBLOCK);

  last_key_time_ = this->get_clock()->now();
  key_timer_ = this->create_wall_timer(5ms, std::bind(&KeyboardController::KeyUpdate, this));
}

KeyboardController::~KeyboardController() {
  // restore termio settings
  tcsetattr(fileno(stdin), TCSANOW, &old_settings_);
  fcntl(fileno(stdin), F_SETFL, f_flags_);
}

char KeyboardController::GetKey() {
  return last_key_.load();
}

void KeyboardController::KeyUpdate() {
  const char key = getchar();
  if (key < 0) {
    if (this->get_clock()->now() - last_key_time_ > 400ms) {
      last_key_.store(0);
    }
  } else {
    last_key_.store(key);
    last_key_time_ = this->get_clock()->now();
  }
}

} // namespace ff