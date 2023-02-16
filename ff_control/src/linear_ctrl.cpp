#include "ff_control/linear_ctrl.hpp"

using namespace std::placeholders;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::FreeFlyerStateStamped;
using ff_msgs::msg::Wrench2D;

namespace ff {

static void State2Vec(const ff_msgs::msg::FreeFlyerState& state, LinearController::StateVec* vec) {
  (*vec)[0] = state.pose.x;
  (*vec)[1] = state.pose.y;
  (*vec)[2] = state.pose.theta;
  (*vec)[3] = state.twist.vx;
  (*vec)[4] = state.twist.vy;
  (*vec)[5] = state.twist.wz;
}

LinearController::LinearController()
  : rclcpp::Node("linear_ctrl_node"),
    WrenchController() {
  state_sub_ = this->create_subscription<FreeFlyerStateStamped>(
    "gt/state", 10, std::bind(&LinearController::StateCallback, this, _1));
}

bool LinearController::GetState(StateVec* state) const {
  std::lock_guard<std::mutex> lock(state_mtx_);

  if (state) {
    *state = state_;
  }

  return state_ready_;
}

void LinearController::SendControl(const StateVec& state_des, const FeedbackMat& K) {
  StateVec state_cur;
  if (!GetState(&state_cur)) {
    RCLCPP_WARN(this->get_logger(), "SendControl ignored, state not yet ready");
    return ;
  }

  StateVec state_delta = state_des - state_cur;
  state_delta[2] = std::remainder(state_delta[2], 2 * M_PI); // wrap angle delta to [-pi, pi]
  const ControlVec u = K * state_delta;

  Wrench2D wrench_world{};
  wrench_world.fx = u[0];
  wrench_world.fy = u[1];
  wrench_world.tz = u[2];
  SetWorldWrench(wrench_world, state_cur[2]);
}

void LinearController::SendControl(const FreeFlyerState& state_des, const FeedbackMat& K) {
  StateVec state_des_vec;
  State2Vec(state_des, &state_des_vec);
  SendControl(state_des_vec, K);
}

void LinearController::StateCallback(const FreeFlyerStateStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mtx_);

  State2Vec(msg->state, &state_);

  state_ready_ = true;
}

} // namespace ff
