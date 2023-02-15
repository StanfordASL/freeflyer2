#pragma once

#include "ff_control/wrench_ctrl.hpp"

#include "ff_msgs/msg/twist2_d.hpp"

namespace ff {

class LinearController : public WrenchController {
 public:
  LinearController();

 protected:
  void SetBodyTwist(const ff_msgs::msg::Twist2D& twist);

  void SetWorldTwist(const ff_msgs::msg::Twist2D& twist);

 private:
};

} // namespace ff
