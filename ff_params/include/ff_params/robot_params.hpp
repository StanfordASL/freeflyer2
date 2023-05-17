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


#pragma once

#include <atomic>
#include <future>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ff
{

struct DynamicsParams
{
  double mass;
  double inertia;
  double radius;
  std::vector<double> CoM_offset;
  std::vector<double> force_const;
};

struct ActuatorParams
{
  double F_max_per_thruster;
  double thrusters_lever_arm;
  double F_body_max;
  double M_body_max;
  double min_inp_percent;
  double max_inp_percent;
  double gamma_min;
  double gamma_max;
};

class RobotParams
{
public:
  DynamicsParams dynamics;
  ActuatorParams actuators;

  explicit RobotParams(rclcpp::Node * node);

  bool Loaded() const;

private:
  std::atomic<bool> loaded_ = false;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;

  void ParamReadyCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
};

}  // namespace ff
