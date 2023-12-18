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


#include "ff_estimate/base_mocap_estimator.hpp"

#include <iostream>
#include <Eigen/Dense>
 
using Eigen::MatrixXd;
using Eigen::VectorXd;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::Pose2DStamped;
using ff_msgs::msg::BinaryCommand;
using ff_msgs::msg::ThrusterCommand;

class KalmanFilter : public ff::BaseMocapEstimator
{
    public:
        KalmanFilter() : ff::BaseMocapEstimator("kalman_filter")
        {
            // kalman filtering on state tracking
            this->declate_parameter("State_Transition", 1);
            this->declare_parameter("Action_Transition", 1);
            this->declare_parameter("predicted_covariance",1);
        }

        struct KalmanFilter {
            Eigen::VectorXd mean;   // mean belief vector: x, y, theta
            Eigen::MatrixXd cov;    // covariance of the belief

            KalmanFilter(const Eigen::VectorXd& initial_mean, const Eigen::MatrixXd& initial_cov)
                : mean(initial_mean), cov(initial_cov) {}
        };

        struct pomdp{
            Eigen::VectorXd Ts;      // Transition vector state
            Eigen::VectorXd Ta;      // Transition vector action
            Eigen::MatrixXd Cov_s;   // state transition covariance
            Eigen::MatrixXd Cov_o;   // observation covariance
            Eigen::MatrixXd Os;      // observation 
            pomdp(const Eigen::VectorXd& initial_Ts,
                const Eigen::VectorXd& initial_Ta,
                const Eigen::MatrixXd& initial_Cov_s,
                const Eigen::MatrixXd& initial_Cov_o,
                const Eigen::MatrixXd& initial_Os)
                : Ts(initial_Ts),
                Ta(initial_Ta),
                Cov_s(initial_Cov_s),
                Cov_o(initial_Cov_o),
                Os(initial_Os) {}
        };

        void update(KalmanFilter& b, pomdp& P, const Eigen::VectorXd& a, const Eigen::VectorXd& o) 
        {
            auto mean_var = Kalman_predict(b, P, a);
            Kalman_update(b, P, o, mean_var.first, mean_var.second);
        }

        std::pair<Eigen::VectorXd, Eigen::MatrixXd> Kalman_predict(const KalmanFilter& b, const pomdp& P, const Eigen::VectorXd& a)
        {
            Eigen::VectorXd mean_p = b.mean;    // predicted mean
            Eigen::MatrixXd var_p = b.cov;      // predicted covariance

            Eigen::MatrixXd Ts = P.Ts, Ta = P.Ta, Cov_s = P.Cov_s;

            Eigen::VectorXd predict_mean = Ts * mean_p + Ta * a;
            Eigen::MatrixXd predict_cov = Ts * var_p * Ts.transpose() + Cov_s;

            return {predict_mean, predict_cov};
        }

        void Kalman_update(KalmanFilter& b, const pomdp& P, const Eigen::VectorXd& o, const Eigen::VectorXd& mean, const Eigen::MatrixXd& var)
        {
            Eigen::MatrixXd Sig_obs = P.Cov_o;
            Eigen::MatrixXd Os = P.Os;

            Eigen::MatrixXd Kalman_gain = var * Sig_obs / (Os * var * Os.transpose() + Sig_obs);
            Eigen::VectorXd mean_b = mean + Kalman_gain * (o - Os * mean);
            Eigen::MatrixXd var_b = (Eigen::MatrixXd::Identity(3, 3) - Kalman_gain * Os) * var;

            b.mean = mean_b;
            b.cov = var_b;
        }

        void EstimateWithPose2D(const Pose2DStamped & pose_stamped) override
        {
            FreeFlyerState state{}; 

            if (prev_state_ready_) {
                const rclcpp::Time now = pose_stamped.header.stamp;
                const rclcpp::Time last = prev_.header.stamp;
                double dt = (now - last).seconds();
                // ignore this frame if it is too close to the last frame
                if (dt < this->get_parameter("min_dt").as_double()) {
                return;
                }
                Eigen::VectorXd pose = Eigen::Map<Eigen::VectorXd>(pose_stamped.pose, 3);
                double Fmax = 0.6;
                double dist = 1;

                Eigen::MatrixXd cov_b(3,3);
                cov_b << 1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0;
                KalmanFilter KF(pose, cov_b);

                pomdp P(
                    Eigen::VectorXd(3) << 0.1, 0.1, 0.1,
                    Eigen::VectorXd(3) << 0.1, 0.1, 0.1,
                    Eigen::MatrixXd::Identity(3, 3),
                    Eigen::MatrixXd::Identity(3, 3),
                    Eigen::MatrixXd(3, 3) << 0.1, 0.1, 0.1
                );

                update(KF, P, action, observation);

                state.pose = Eigen::VectorXd(3) << KF.mean[0], KF.mean[1], KF.mean[2];

                // finite difference
                double vx = (state.pose.x - prev_.state.pose.x) / dt;
                double vy = (state.pose.y - prev_.state.pose.y) / dt;
                // wrap angle delta to [-pi, pi]
                double dtheta = std::remainder(state.pose.theta - prev_.state.pose.theta, 2 * M_PI);
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
}