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
#include <chrono>

#include "ff_control/keyboard_ctrl.hpp"

using namespace std::chrono_literals;

namespace ff
{

KeyboardController::KeyboardController()
: rclcpp::Node("keyboard_ctrl_node")
{
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

KeyboardController::~KeyboardController()
{
  // restore termio settings
  tcsetattr(fileno(stdin), TCSANOW, &old_settings_);
  fcntl(fileno(stdin), F_SETFL, f_flags_);
}

char KeyboardController::GetKey()
{
  return last_key_.load();
}

void KeyboardController::KeyUpdate()
{
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

}  // namespace ff
