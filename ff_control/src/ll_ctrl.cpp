#include "ff_control/ll_ctrl.hpp"

using std_msgs::msg::Float64MultiArray;
using std_msgs::msg::Float64;

namespace ff {

LowLevelController::LowLevelController(const std::string& node_name)
  : rclcpp::Node(node_name),
    p_(this) {
  thruster_pub_ = this->create_publisher<Float64MultiArray>("commands/duty_cycle", 10);
  wheel_pub_ = this->create_publisher<Float64>("commands/velocity", 10);
}

void LowLevelController::SetThrustDutyCycle(const Float64MultiArray& msg) {
  thruster_pub_->publish(msg);
}

void LowLevelController::SetWheelVelocity(const Float64& msg) {
  wheel_pub_->publish(msg);
}

} // namespace ff
