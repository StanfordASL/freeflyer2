#include "ff_control/ll_ctrl.hpp"

using ff_msgs::msg::ThrusterCommand;
using ff_msgs::msg::WheelVelCommand;

namespace ff {

LowLevelController::LowLevelController()
  : rclcpp::Node("ll_ctrl_node"),
    p_(this) {
  thruster_pub_ = this->create_publisher<ThrusterCommand>("ctrl/duty_cycle", 10);
  wheel_pub_ = this->create_publisher<WheelVelCommand>("ctrl/velocity", 10);
}

void LowLevelController::SetThrustDutyCycle(const std::array<double, 8>& duty_cycle) {
  ThrusterCommand msg{};
  msg.header.stamp = this->get_clock()->now();
  msg.duty_cycle = duty_cycle;

  thruster_pub_->publish(msg);
}

void LowLevelController::SetWheelVelocity(const double& velocity) {
  WheelVelCommand msg{};
  msg.header.stamp = this->get_clock()->now();
  msg.velocity = velocity;

  wheel_pub_->publish(msg);
}

} // namespace ff
