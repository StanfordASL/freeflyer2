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


#pragma once

#include <termios.h>

#include <atomic>

#include <rclcpp/rclcpp.hpp>

namespace ff
{

class KeyboardController : virtual public rclcpp::Node
{
public:
  KeyboardController();
  ~KeyboardController();

protected:
  /**
   * @brief get the last key press
   *
   * @return the last key, 0 if nothing is pressed within 0.4 seconds
   */
  char GetKey();

private:
  // termio settings backup
  struct termios old_settings_;
  int f_flags_;

  // sticky key
  std::atomic<char> last_key_ = 0;
  rclcpp::Time last_key_time_;

  rclcpp::TimerBase::SharedPtr key_timer_;

  void KeyUpdate();
};

}  // namespace ff
