#include "ff_params/robot_params.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ff {

RobotParams::RobotParams(rclcpp::Node* node)
  : loaded_(false),
    param_client_(std::make_shared<rclcpp::AsyncParametersClient>(
      node, std::string(node->get_namespace()) + "/robot_params_node")) {
  while (!param_client_->service_is_ready()) {
    param_client_->wait_for_service(5s);
    RCLCPP_INFO(node->get_logger(), "parameter service not ready, retrying...");
  }

  param_client_->get_parameters({
    "dynamics.mass",
    "dynamics.inertia",
    "dynamics.radius",
    "dynamics.CoM_offset",
    "dynamics.force_const",
    "actuators.F_max_per_thruster",
    "actuators.thrusters_lever_arm",
    "actuators.F_body_max",
    "actuators.M_body_max",
    "actuators.min_inp_percent",
    "actuators.max_inp_percent",
    "actuators.gamma_min",
    "actuators.gamma_max",
  }, std::bind(&RobotParams::ParamReadyCallback, this, _1));
}

bool RobotParams::Loaded() const {
  return loaded_.load();
}

void RobotParams::ParamReadyCallback(std::shared_future<std::vector<rclcpp::Parameter>> future) {
  const auto params = future.get();

  this->dynamics.mass = params[0].as_double();
  this->dynamics.inertia = params[1].as_double();
  this->dynamics.radius = params[2].as_double();
  this->dynamics.CoM_offset = params[3].as_double_array();
  this->dynamics.force_const = params[4].as_double_array();

  this->actuators.F_max_per_thruster = params[5].as_double();
  this->actuators.thrusters_lever_arm = params[6].as_double();
  this->actuators.F_body_max = params[7].as_double();
  this->actuators.M_body_max = params[9].as_double();
  this->actuators.min_inp_percent = params[9].as_double();
  this->actuators.max_inp_percent = params[10].as_double();
  this->actuators.gamma_min = params[11].as_double();
  this->actuators.gamma_max = params[12].as_double();

  loaded_.store(true);
}

} // namespace ff
