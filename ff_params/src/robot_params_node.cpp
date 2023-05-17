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


#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class RobotParamsNode : public rclcpp::Node
{
public:
  explicit RobotParamsNode(const std::string & node_name)
  : rclcpp::Node(node_name)
  {
    // dynamics
    this->declare_parameters<double>(
      "dynamics", {
      {"mass", 16.},      // mass [kg]
      {"inertia", 0.18},  // inertia at CoM (no payload) [kg*m^2]
      {"radius", 0.1},    // robot radius, in [m]
    });
    this->declare_parameters<std::vector<double>>(
      "dynamics", {
      {"CoM_offset", {0., 0.}},         // center of mass, in body frame [m]
      {"force_const", {0.005, 0.005}},  // constant force (e.g. table tilt), in world frame [N]
    });

    // actuators
    this->declare_parameters<double>(
      "actuators", {
      {"F_max_per_thruster", 0.2},       // max force per thruster, in [N]
      {"thrusters_lever_arm", 0.11461},  // lever arm, in [m]
      {"F_body_max", 0.4},        // body frame force max, in [N]
      {"M_body_max", 0.05},       // body frame moment / torque max, in [Nm]
      {"min_inp_percent", 0.05},  // in [0,0.15]  - inputs are zero close to min input
      {"max_inp_percent", 0.95},  // in [0.8,1.0] - inflexion pt for input sat (PWMs overlapping)
      {"gamma_min", 500.},        // in [100,1000] - sharpness for inp saturation close to min inp
      {"gamma_max", 500.},        // in [100,1000] - sharpness for inp saturation close to max inp
    });

    timer_ = this->create_wall_timer(1h, []() {});
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotParamsNode>("robot_params_node"));
  rclcpp::shutdown();
  return 0;
}
