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
  /**
   * @brief get the last key press
   *
   * @return the last key, 0 if nothing is pressed within 1 second
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

} // namespace ff
