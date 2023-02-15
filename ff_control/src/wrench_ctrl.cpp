#include "ff_control/wrench_ctrl.hpp"

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ff {

void WrenchController::SetWrench(const geometry_msgs::msg::Wrench& msg, bool use_wheel) {
  if (use_wheel) {
    // TODO(alvin): support wheel velocity control in the future or remove?
    RCLCPP_ERROR(this->get_logger(), "SetWrench failed: use_wheel not implemented");
  } else {
    (void)msg;
  }
}

} // namespace ff
