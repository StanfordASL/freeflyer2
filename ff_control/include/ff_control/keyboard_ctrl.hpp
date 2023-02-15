#pragma once

#include <atomic>
#include <termio.h>

#include <rclcpp/rclcpp.hpp>

namespace ff {

class KeyboardController : virtual public rclcpp::Node {
 public:
  KeyboardController();
  ~KeyboardController();

 protected:
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

} // namespace ff
