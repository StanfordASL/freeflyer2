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
#include "ff_params/include/ff_params/robot_params.hpp"

using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::Pose2DStamped;
using ff_msgs::msg::BinaryCommand;
using ff_msgs::msg::ThrusterCommand;


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

void Pose2DEstimator::EstimateWithPose2D_Filtered(const Pose2DStamped & pose_stamped, const ThrusterCommand & action)
{
  FreeFlyerState state{};
  RobotParams freeflyer;

  state.pose = pose_stamped.pose;
  if (prev_state_ready_) {
    const rclcpp::Time now = pose_stamped.header.stamp;
    const rclcpp::Time last = prev_.header.stamp;
    double dt = (now - last).seconds();
    '''
    Kalman Filter:
    need action a, predicted state : belief b, observed future state : belief b_
    '''
    Fmax = freeflyer.actuators.F_body_max;
    dist = freeflyer.actuators.thrusters_lever_arm;

    // prediction step using the action and the previous step
    FreeFlyerState prediction{}; // create new state prediction
    
    // forces in body frame
    F_x = Fmax * ((action[2] + action[5]) - (action[1] + action[6]));
    F_y = Fmax * ((action[4] + action[7]) - (action[0] + action[3]));
    M = (dist * Fmax) * ((action[1]+action[3]+action[5]+action[7])-(action[0]+action[2]+action[4]+action[6]));

    R = get_rotmatrix_body_to_world(prev_.state.pose.theta);
    F_worldFrame = matmul(R, [F_x, F_y]);

    ucur = [F_x, F_y, M];

    m = freeflyer.dynamics.mass;
    J = freeflyer.dynamics.inertia;
    p0 = freeflyer.dynamics.CoM_offset;
    F_tilt = freeflyer.dynamics.force_const;

    f = std::array<double, 6>;
    f[0:2] = [prev_.state.twist.vx, prev_.state.twist.vy];
    f[2] = prev_.state.twist.wz;
    thetaddot = (M - F[1] * p0[0] + F[0] * p0[1]) / J;
    f[3:5] = matmul(R, (F / m - (thetaddot * np.array([-p0[1], p0[0]]) - prev_.state.twist.wz**2 * p0)));
    f[5] = thetaddot;

    prediction.pose.x = prev_.state.pose.x + dt * f[0];
    prediction.pose.y = prev_.state.pose.y + dt * f[1];
    prediction.pose.theta = prev_.state.pose.theta + dt * f[2];
    prediction.twist.vx = prev_.state.twist.vx + dt * f[3];
    prediction.twist.vy = prev_.state.twist.vy + dt * f[4];
    prediction.twist.wz = prev_.state.twist.wz + dt * f[5];

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

  // have the prediction and state. State being the observed believed state. might be noisy


  SendStateEstimate(state);
} 
}
class KalmanFilter{
  public:
    double mean;
    double covariance;
    void update();
};
KalmanFilter::update(std::array<double,6> & predict, std::array<double,6> & obs_state){
  
}

std::array<std::array<double, 2>, 2> get_rotmatrix_body_to_world(double theta) {
    std::array<std::array<double, 2>, 2> R = {{
        {std::cos(theta), -std::sin(theta)},
        {std::sin(theta), std::cos(theta)}
    }};
    return R;
}
std::array<double, 2> matmul(const std::array<std::array<double, 2>, 2>& R, const std::array<double, 2>& F_bodyFrame) {
    std::array<double, 2> F_worldFrame;
    
    F_worldFrame[0] = R[0][0] * F_bodyFrame[0] + R[0][1] * F_bodyFrame[1];
    F_worldFrame[1] = R[1][0] * F_bodyFrame[0] + R[1][1] * F_bodyFrame[1];

    return F_worldFrame;
}

std::array<double, 6>, f_dynamics_continuous_time(std::array<double, 6> & x, std::array<double,6>){
  m = 

}  // namespace ff