#include "ff_control/pwm_ctrl.hpp"

using namespace std::chrono_literals;

namespace ff {

PWMController::PWMController()
  : rclcpp::Node("pwm_ctrl_node"),
    period_(1.0s / this->declare_parameter("pwm_frequency", 10.0)) {
  period_timer_ = this->create_wall_timer(period_, std::bind(&PWMController::PeriodCallback, this));
}

void PWMController::SetThrustDutyCycle(const std::array<double, 8> & duty_cycle)
{
  duty_cycle_ = duty_cycle;
}

void PWMController::PeriodCallback() {
  std::array<bool, 8> thrusts = {0};

  for (size_t i = 0; i < thrusts.size(); ++i) {
    if (duty_cycle_[i] > 0) {
      thrusts[i] = true;
      switch_timers_[i] = this->create_wall_timer(
        duty_cycle_[i] * period_,
        [this, i]() {SwitchCallback(i);}
      );
    }
  }

  SetAllThrusts(thrusts);
}

void PWMController::SwitchCallback(size_t idx) {
  if (idx >= duty_cycle_.size()) {
    RCLCPP_ERROR(this->get_logger(), "PWM idx out of range");
    return;
  }

  if (!switch_timers_[idx]) {
    RCLCPP_ERROR(this->get_logger(), "encountered unitialized pointer");
    return;
  }

  switch_timers_[idx]->cancel();
  SetThrust(idx, false);
}

}  // namespace ff
