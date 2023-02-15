#pragma once

#include <string>

#include <geometry_msgs/msg/wrench.hpp>

#include "ff_control/ll_ctrl.hpp"

namespace ff {

class WrenchController : public LowLevelController  {
 protected:
  void SetWrench(const geometry_msgs::msg::Wrench& wrench, bool use_wheel = false);
};

} // namespace ff
