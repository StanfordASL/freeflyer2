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

#include <Eigen/Dense>
#include <mutex>

#include "ff_control/wrench_ctrl.hpp"
#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

namespace ff
{

class LinearController : public WrenchController
{
public:
  using StateVec = Eigen::Matrix<double, 6, 1>;     // [x, y, theta, vx, vy, wz]
  using ControlVec = Eigen::Matrix<double, 3, 1>;   // [fx, fy, tz]
  using FeedbackMat = Eigen::Matrix<double, 3, 6>;

  LinearController();

protected:
  /**
   * @brief check whether the state is available
   *
   * @return true if the first state message has arrived.
   */
  bool StateIsReady() const;

  /**
   * @brief get the current state of the robot
   *
   * @param state output StateVec pointer
   *
   * @return true if state is ready
   */
  bool GetState(StateVec * state) const;

  /**
   * @brief get the current state of the robot
   *
   * @param state output FreeFlyerState pointer
   *
   * @return true if state is ready
   */
  bool GetState(ff_msgs::msg::FreeFlyerState * state) const;

  /**
   * @brief send linear feedback control command
   *
   * @param state_des desired state
   * @param K         feedback control matrix (i.e. u = Kx)
   */
  void SendControl(const StateVec & state_des, const FeedbackMat & K);

  /**
   * @brief send linear feedback control command
   *
   * @param state_des desired state
   * @param K         feedback control matrix (i.e. u = Kx)
   */
  void SendControl(const ff_msgs::msg::FreeFlyerState & state_des, const FeedbackMat & K);

  /**
   * @brief called when the first state message is received
   */
  virtual void StateReadyCallback();

private:
  StateVec state_;
  bool state_ready_ = false;
  mutable std::mutex state_mtx_;

  rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_sub_;

  void StateCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg);
};

}  // namespace ff
