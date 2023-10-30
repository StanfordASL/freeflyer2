#include "ff_control/Lowll_ctrl.hpp"

using ff_msgs::msg::ThrusterCommand;
using ff_msgs::msg::WheelVelCommand;

namespace ff
{

LowLowLevelController::LowLowLevelController()
: rclcpp::Node("Lowll_ctrl_node"),
  p_(this)
{
  thruster_pub_ = this->create_publisher<ThrusterCommandBinary>("ctrl/duty_cycle", 10);
//   wheel_pub_ = this->create_publisher<WheelVelCommand>("ctrl/velocity", 10);
}

void LowLowLevelController::SetBinaryDutyCycle(const std::array<bool, 8> & duty_cycle)
{
  ThrusterCommand msg{};
  msg.header.stamp = this->get_clock()->now();
  msg.binary_command = duty_cycle;

  thruster_pub_->publish(msg);
}

void LowLevelController::SetWheelVelocity(const double & velocity)
{
  WheelVelCommand msg{};
  msg.header.stamp = this->get_clock()->now();
  msg.velocity = velocity;

  wheel_pub_->publish(msg);
}

}  // namespace ff