#include "ff_control/wrench_ctrl.hpp"

using ff_msgs::msg::Wrench2D;

namespace ff {

void WrenchController::SetBodyWrench(const Wrench2D& wrench_body, bool use_wheel) {
  if (use_wheel) {
    // TODO(alvin): support wheel velocity control in the future or remove?
    RCLCPP_ERROR(this->get_logger(), "SetWrench failed: use_wheel not implemented");
  } else {
    std::array<double, 8> duty_cycle;
    duty_cycle.fill(0);

    // convert force
    const double u_Fx = wrench_body.fx / (2 * p_.actuators.F_max_per_thruster);
    const double u_Fy = wrench_body.fy / (2 * p_.actuators.F_max_per_thruster);
    if (u_Fx > 0) {
      duty_cycle[2] = u_Fx;
      duty_cycle[5] = u_Fx;
    } else {
      duty_cycle[1] = -u_Fx;
      duty_cycle[6] = -u_Fx;
    }
    if (u_Fy > 0) {
      duty_cycle[4] = u_Fy;
      duty_cycle[7] = u_Fy;
    } else {
      duty_cycle[0] = -u_Fy;
      duty_cycle[3] = -u_Fy;
    }

    // convert torque
    const double u_M = wrench_body.tz /
      (4 * p_.actuators.F_max_per_thruster * p_.actuators.thrusters_lever_arm);
    if (u_M > 0) {
      for (const int& i : {1, 3, 5, 7}) {
        duty_cycle[i] += u_M;
      }
    } else {
      for (const int& i : {0, 2, 4, 6}) {
        duty_cycle[i] += -u_M;
      }
    }

    // clip to [0, 1]
    for (int i = 0; i < 8; ++i) {
      duty_cycle[i] = std::max(std::min(1., duty_cycle[i]), 0.);
    }

    this->SetThrustDutyCycle(duty_cycle);
  }
}

void WrenchController::SetWorldWrench(const ff_msgs::msg::Wrench2D& wrench_world, double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  ff_msgs::msg::Wrench2D wrench_body = wrench_world;
  wrench_body.fx = cos_theta * wrench_world.fx + sin_theta * wrench_world.fy;
  wrench_body.fy = -sin_theta * wrench_world.fx + cos_theta * wrench_world.fy;

  SetBodyWrench(wrench_body);
}

} // namespace ff
