//Dock node and Filter

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose2_d_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <optional>

using namespace std

class ConstantVelKF {
    public:
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        double MAX_DT;

        ConstantAccelKF(Eigen::VectorXd x0, Eigen::MatrixXd P0, int dim = 3, int angle_idx = 2)
            : x(x0), P(P0), dim(dim), angle_idx(angle_idx) {
            Q = Eigen::MatrixXd::Identity(2 * dim, 2 * dim);
            R = Eigen::MatrixXd::Identity(dim, dim) * 2.4445e-3, 1.2527e-3, 4.0482e-3;
            MAX_DT = 1e-3;
        }

        void process_update(double dt) {
            if (dt <= 0.) {
                return;
            }

            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2 * dim, 2 * dim);
            A.block(0, dim, dim, dim) = Eigen::MatrixXd::Identity(dim, dim) * dt;

            x = A * x;
            P = A * P * A.transpose() + Q * dt;
        }

        void measurement_update(Eigen::VectorXd z) {
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, 2 * dim);
            H.block(0, 0, dim, dim) = Eigen::MatrixXd::Identity(dim, dim);

            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            Eigen::VectorXd y = z - H * x;
            y(angle_idx) = wrap_angle(y(angle_idx));

            x += K * y;
            P -= K * H * P;
        }

        double wrap_angle(double theta) {
            return atan2(sin(theta), cos(theta));
        }

    private:
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
        int dim;
        int angle_idx;
};


#include "ff_estimate/base_mocap_estimator.hpp"

class ConstVelKalmanFilterNode : public ff::BaseMocapEstimator {
    public:
        ConstVelKalmanFilterNode() : ff::BaseMocapEstimator("const_vel_kalman_filter_node") {
            this->declare_parameter("min_dt", 0.005);
            this->target_pose = Pose2D;
        }

    void EstimatewithPose2D(const Pose2DStamped & pose_stamped) override {
        FreeFlyerState state{};
        Pose2D pose2d{};

        state.pose = pose_stamped.pose;
        if (prev_state_ready_) {
            const rclcpp::Time now = pose_stamped.header.stamp;
            const rclcpp::Time last = prev_.header.stamp;
            double dt = (now - last).seconds();

            if (dt < (this->get_parameter("min_dt").as_double())) {
                return;
            }

            target_pose.pose.x = pose2d.pose.position.x;
            target_pose.pose.y = pose2d.pose.position.y;
            target_pose.pose.theta = pose2d.pose.position.theta;

            state.header = est_state.header
            state.state.twist = pose_stamped.state.twist;
            state.state.pose.x = this.target_pose.x;
            state.state.pose.y = this.target_pose.y;
            state.state.pose.theta = this.target_pose.theta;
        } else {
            prev_state_ready_ = true;
        }

        prev_.state = state;
        prev_.header = pose_stamped.header;

        SendStateEstimate(state);
    }

    private:
        geometry_msgs::msg::TwistStamped;
        geometry_msgs::msg::Pose2DStamped;
        ff_msgs::msg::FreeFlyerStateStamped prev_;
        bool prev_state_ready_ = false;
        geometry_msgs::msg::Pose2D;

        /* void target_loop() {
            if (!target_pose_.has_value()) {
                return;
            }

            auto target = std::make_shared<geometry_msgs::msg::TwistStamped>();
            target->header.stamp = now();
            target->twist.linear.x = target_pose_->x;
            target->twist.linear.y = target_pose_->y - 0.5;
            target->twist.angular.z = target_pose_->theta;

            target_pub_->publish(target);
        }

        void est_callback(const geometry_msgs::msg::Pose2DStamped::SharedPtr cv_pose) {
            if (!target_pose_.has_value()) {
                return;
            }

            auto state = std::make_shared<geometry_msgs::msg::TwistStamped>();
            state->header = cv_pose->header;
            state->twist = cv_pose->pose;
            state->twist.linear.x += target_pose_->x;
            state->twist.linear.y += target_pose_->y;
            state->twist.angular.z += target_pose_->theta;

            state_pub_->publish(state);
        }

        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
            if (!target_pose_.has_value()) {
                target_pose_ = geometry_msgs::msg::Pose2D();
            }

            target_pose_->x = target_pose->pose.position.x;
            target_pose_->y = target_pose->pose.position.y;
            target_pose_->theta = M_PI / 2.0;
        } */
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConstantVelKalmanFilterNode>());
    rclcpp::shutdown();
}
