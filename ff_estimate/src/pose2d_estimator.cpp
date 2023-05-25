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


#include "ff_estimate/pose2d_estimator.hpp"
#include "ff_msgs/msg/free_flyer_state.hpp"

using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::Pose2DStamped;

namespace ff
{

Pose2DEstimator::Pose2DEstimator(const std::string & node_name)
: BaseEstimator(node_name)
{
  // perform simple 1st order IIR lowpass filter on derivate estimates
  // coefficient in [0, 1) (higher coefficient filters better with larger delay)
  this->declare_parameter("lowpass_coeff", .95);
  this->declare_parameter("min_dt", 0.005);
}

void Pose2DEstimator::EstimateWithPose2D(const Pose2DStamped & pose_stamped)
{
  FreeFlyerState state{};

  state.pose = pose_stamped.pose;
  if (prev_state_ready_) {
    const rclcpp::Time now = pose_stamped.header.stamp;
    const rclcpp::Time last = prev_.header.stamp;
    double dt = (now - last).seconds();

    // ignore this frame if it is too close to the last frame
    if (dt < this->get_parameter("min_dt").as_double()) {
      return;
    }

    // finite difference
    double vx = (pose_stamped.pose.x - prev_.state.pose.x) / dt;
    double vy = (pose_stamped.pose.y - prev_.state.pose.y) / dt;
    // wrap angle delta to [-pi, pi]
    double dtheta = std::remainder(pose_stamped.pose.theta - prev_.state.pose.theta, 2 * M_PI);
    double wz = dtheta / dt;

    double alpha = this->get_parameter("lowpass_coeff").as_double();
    if (alpha < 0 || alpha >= 1) {
      RCLCPP_ERROR(this->get_logger(), "IIR filter disabled: invalid coefficient %f", alpha);
      alpha = 0;
    }
    state.twist.vx = alpha * prev_.state.twist.vx + (1. - alpha) * vx;
    state.twist.vy = alpha * prev_.state.twist.vy + (1. - alpha) * vy;
    state.twist.wz = alpha * prev_.state.twist.wz + (1. - alpha) * wz;
  } else {
    prev_state_ready_ = true;
  }

  prev_.state = state;
  prev_.header = pose_stamped.header;

  SendStateEstimate(state);
}

}  // namespace ff
