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


#include <cmath>

#include "ff_control/wrench_ctrl.hpp"

using ff_msgs::msg::Wrench2D;

namespace ff
{

WrenchController::WrenchController()
: rclcpp::Node("wrench_ctrl_node"),
  PWMController() {}

void WrenchController::SetBodyWrench(const Wrench2D & wrench_body, bool use_wheel)
{
  if (use_wheel) {
    // TODO(alvin): support wheel velocity control in the future or remove?
    RCLCPP_ERROR(this->get_logger(), "SetWrench failed: use_wheel not implemented");
  } else {
    std::array<double, 8> duty_cycle;
    duty_cycle.fill(0);

    const Wrench2D wrench_body_clipped = ClipWrench(wrench_body);

    // convert force
    const double u_Fx = wrench_body_clipped.fx / (2 * p_.actuators.F_max_per_thruster);
    const double u_Fy = wrench_body_clipped.fy / (2 * p_.actuators.F_max_per_thruster);
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
    const double u_M = wrench_body_clipped.tz /
      (4 * p_.actuators.F_max_per_thruster * p_.actuators.thrusters_lever_arm);
    if (u_M > 0) {
      for (const int & i : {1, 3, 5, 7}) {
        duty_cycle[i] += u_M;
      }
    } else {
      for (const int & i : {0, 2, 4, 6}) {
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

void WrenchController::SetWorldWrench(const ff_msgs::msg::Wrench2D & wrench_world, double theta)
{
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  ff_msgs::msg::Wrench2D wrench_body = wrench_world;
  wrench_body.fx = cos_theta * wrench_world.fx + sin_theta * wrench_world.fy;
  wrench_body.fy = -sin_theta * wrench_world.fx + cos_theta * wrench_world.fy;

  SetBodyWrench(wrench_body);
}

Wrench2D WrenchController::ClipWrench(const Wrench2D & wrench) const
{
  Wrench2D wrench_clipped;
  const double force = std::sqrt(wrench.fx * wrench.fx + wrench.fy * wrench.fy);
  const double force_scale = std::max(force / p_.actuators.F_body_max, 1.0);
  const double torque_scale = std::max(std::abs(wrench.tz) / p_.actuators.M_body_max, 1.0);

  wrench_clipped.fx = wrench.fx / force_scale;
  wrench_clipped.fy = wrench.fy / force_scale;
  wrench_clipped.tz = wrench.tz / torque_scale;

  return wrench_clipped;
}

}  // namespace ff
