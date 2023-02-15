#pragma once

#include <atomic>
#include <future>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ff {

struct DynamicsParams {
  double mass;
  double inertia;
  double radius;
  std::vector<double> CoM_offset;
  std::vector<double> force_const;
};

struct ActuatorParams {
  double F_max_per_thruster;
  double thrusters_lever_arm;
  double F_body_max;
  double M_body_max;
  double min_inp_percent;
  double max_inp_percent;
  double gamma_min;
  double gamma_max;
};

class RobotParams {
 public:
  DynamicsParams dynamics;
  ActuatorParams actuators;

  RobotParams(rclcpp::Node* node);

  bool Loaded() const;

 private:
  std::atomic<bool> loaded_ = false;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;

  void ParamReadyCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
};

} // namespace ff
