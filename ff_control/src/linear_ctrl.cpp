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


#include "ff_control/linear_ctrl.hpp"

using namespace std::placeholders;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::FreeFlyerStateStamped;
using ff_msgs::msg::Wrench2D;

namespace ff
{

static void State2Vec(const ff_msgs::msg::FreeFlyerState & state, LinearController::StateVec * vec)
{
  if (vec) {
    (*vec)[0] = state.pose.x;
    (*vec)[1] = state.pose.y;
    (*vec)[2] = state.pose.theta;
    (*vec)[3] = state.twist.vx;
    (*vec)[4] = state.twist.vy;
    (*vec)[5] = state.twist.wz;
  }
}

static void Vec2State(const LinearController::StateVec & vec, ff_msgs::msg::FreeFlyerState * state)
{
  if (state) {
    state->pose.x = vec[0];
    state->pose.y = vec[1];
    state->pose.theta = vec[2];
    state->twist.vx = vec[3];
    state->twist.vy = vec[4];
    state->twist.wz = vec[5];
  }
}

LinearController::LinearController()
: rclcpp::Node("linear_ctrl_node"),
  WrenchController()
{
  this->declare_parameter("state_channel", "est/state");
  state_sub_ = this->create_subscription<FreeFlyerStateStamped>(
    this->get_parameter("state_channel").as_string(),
    10,
    std::bind(&LinearController::StateCallback, this, _1));
}

bool LinearController::StateIsReady() const
{
  std::lock_guard<std::mutex> lock(state_mtx_);

  return state_ready_;
}

bool LinearController::GetState(StateVec * state) const
{
  std::lock_guard<std::mutex> lock(state_mtx_);

  if (state) {
    *state = state_;
  }

  return state_ready_;
}

bool LinearController::GetState(FreeFlyerState * state) const
{
  std::lock_guard<std::mutex> lock(state_mtx_);

  if (state) {
    Vec2State(state_, state);
  }

  return state_ready_;
}

void LinearController::SendControl(const StateVec & state_des, const FeedbackMat & K)
{
  StateVec state_cur;
  if (!GetState(&state_cur)) {
    RCLCPP_WARN(this->get_logger(), "SendControl ignored, state not yet ready");
    return;
  }

  StateVec state_delta = state_des - state_cur;
  state_delta[2] = std::remainder(state_delta[2], 2 * M_PI);  // wrap angle delta to [-pi, pi]
  const ControlVec u = K * state_delta;

  Wrench2D wrench_world{};
  wrench_world.fx = u[0];
  wrench_world.fy = u[1];
  wrench_world.tz = u[2];
  SetWorldWrench(wrench_world, state_cur[2]);
}

void LinearController::SendControl(const FreeFlyerState & state_des, const FeedbackMat & K)
{
  StateVec state_des_vec;
  State2Vec(state_des, &state_des_vec);
  SendControl(state_des_vec, K);
}

void LinearController::StateReadyCallback() {}

void LinearController::StateCallback(const FreeFlyerStateStamped::SharedPtr msg)
{
  bool run_callback = false;

  // locked session for state_ and state_ready_ access
  {
    std::lock_guard<std::mutex> lock(state_mtx_);

    State2Vec(msg->state, &state_);

    if (!state_ready_) {
      state_ready_ = true;
      run_callback = true;
    }
  }

  if (run_callback) {
    StateReadyCallback();
  }
}

}  // namespace ff
