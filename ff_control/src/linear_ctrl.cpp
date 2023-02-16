#include "ff_control/linear_ctrl.hpp"

using namespace std::placeholders;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::FreeFlyerStateStamped;
using ff_msgs::msg::Wrench2D;

namespace ff {

LinearController::LinearController()
  : rclcpp::Node("linear_ctrl_node"),
    WrenchController() {
  state_sub_ = this->create_subscription<FreeFlyerStateStamped>(
    "gt/state", 10, std::bind(&LinearController::StateCallback, this, _1));
}

LinearController::StateVec LinearController::GetState() const {
  std::lock_guard<std::mutex> lock(state_mtx_);
  return state_;
}

void LinearController::State2Vec(const ff_msgs::msg::FreeFlyerState& state, StateVec* vec) const {
  (*vec)[0] = state.pose.x;
  (*vec)[1] = state.pose.y;
  (*vec)[2] = state.pose.theta;
  (*vec)[3] = state.twist.vx;
  (*vec)[4] = state.twist.vy;
  (*vec)[5] = state.twist.wz;
}

void LinearController::SendControl(const StateVec& state_des, const FeedbackMat& K) {
  const StateVec state_cur = GetState();
  const ControlVec u = K * (state_des - state_cur);

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
}

} // namespace ff
