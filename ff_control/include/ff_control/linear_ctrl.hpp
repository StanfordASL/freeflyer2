#pragma once

#include "ff_control/wrench_ctrl.hpp"

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

#include <mutex>
#include <Eigen/Dense>

namespace ff {

class LinearController : public WrenchController {
  using StateVec = Eigen::Vector<double, 6>;        // [x, y, theta, vx, vy, wz]
  using ControlVec = Eigen::Vector<double, 3>;      // [fx, fy, tz]
  using FeedbackMat = Eigen::Matrix<double, 3, 6>;

 public:
  LinearController();

 protected:
  /**
   * @brief get the current state of the robot
   *
   * @return state in Vector6d
   */
  StateVec GetState() const;

  /**
   * @brief convert FreeFlyerState to StateVec
   *
   * @param state FreeFlyerState input
   * @param vec   StateVec output
   */
  void State2Vec(const ff_msgs::msg::FreeFlyerState& state, StateVec* vec) const;

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

 private:
  StateVec state_;
  mutable std::mutex state_mtx_;

  rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_sub_;

  void StateCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg);
};

} // namespace ff
