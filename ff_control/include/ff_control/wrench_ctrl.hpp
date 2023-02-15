#pragma once

#include <string>

#include "ff_control/ll_ctrl.hpp"

#include "ff_msgs/msg/wrench2_d.hpp"

namespace ff {

class WrenchController : public LowLevelController  {
 public:
   WrenchController();

 protected:
  void SetBodyWrench(const ff_msgs::msg::Wrench2D& wrench_body, bool use_wheel = false);

  void SetWorldWrench(const ff_msgs::msg::Wrench2D& wrench_world, double theta);
};

} // namespace ff
