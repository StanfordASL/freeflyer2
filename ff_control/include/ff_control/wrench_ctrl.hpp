#pragma once

#include <string>

#include "ff_control/ll_ctrl.hpp"

#include "ff_msgs/msg/wrench2_d.hpp"

namespace ff {

class WrenchController : public LowLevelController  {
 public:
   WrenchController();

 protected:
  /**
   * @brief set wrench in body frame
   *
   * @param wrench_body wrench in body frame
   * @param use_wheel   set to true to use the inertial wheel (TODO(alvin): unsupported)
   */
  void SetBodyWrench(const ff_msgs::msg::Wrench2D& wrench_body, bool use_wheel = false);

  /**
   * @brief set wrench in world frame
   *
   * @param wrench_world  wrench in world frame
   * @param theta         rotational state of the freeflyer
   */
  void SetWorldWrench(const ff_msgs::msg::Wrench2D& wrench_world, double theta);

 private:
  ff_msgs::msg::Wrench2D ClipWrench(const ff_msgs::msg::Wrench2D& wrench) const;
};

} // namespace ff
