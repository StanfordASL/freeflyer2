#pragma once

#include "ff_control/wrench_ctrl.hpp"

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

#include <mutex>
#include <Eigen/Dense>

namespace ff {

class LinearController : public WrenchController {
 public:
  static constexpr int STATE_DIM = 6;
  static constexpr int CONTROL_DIM = 3;
  using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;       // [x, y, theta, vx, vy, wz]
  using ControlVec = Eigen::Matrix<double, CONTROL_DIM, 1>;   // [fx, fy, tz]
  using FeedbackMat = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;

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
  bool GetState(StateVec* state) const;

  /**
   * @brief get the current state of the robot
   *
   * @param state output FreeFlyerState pointer
   *
   * @return true if state is ready
   */
  bool GetState(ff_msgs::msg::FreeFlyerState* state) const;

  /**
   * @brief send linear feedback control command
   *
   * @param state_des desired state
   * @param K         feedback control matrix (i.e. u = Kx)
   */
  void SendControl(const StateVec& state_des, const FeedbackMat& K);

  /**
   * @brief send linear feedback control command
   *
   * @param state_des desired state
   * @param K         feedback control matrix (i.e. u = Kx)
   */
  void SendControl(const ff_msgs::msg::FreeFlyerState& state_des, const FeedbackMat& K);

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

} // namespace ff
