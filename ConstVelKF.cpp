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

class ConstantVelKF {
public:
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    double MAX_DT;

    ConstantAccelKF(Eigen::VectorXf x0, Eigen::MatrixXf P0, int dim = 3, int angle_idx = 2)
        : x(x0), P(P0), dim(dim), angle_idx(angle_idx) {
        Q = Eigen::MatrixXf::Identity(2 * dim, 2 * dim);
        R = Eigen::MatrixXf::Identity(dim, dim) * 2.4445e-3, 1.2527e-3, 4.0482e-3;
        MAX_DT = 1e-3;
    }

    void process_update(double dt) {
        if (dt <= 0.) {
            return;
        }

        Eigen::MatrixXf A = Eigen::MatrixXf::Identity(2 * dim, 2 * dim);
        A.block(0, dim, dim, dim) = Eigen::MatrixXf::Identity(dim, dim) * dt;

        x = A * x;
        P = A * P * A.transpose() + Q * dt;
    }

    void measurement_update(Eigen::VectorXf z) {
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(dim, 2 * dim);
        H.block(0, 0, dim, dim) = Eigen::MatrixXf::Identity(dim, dim);

        Eigen::MatrixXf S = H * P * H.transpose() + R;
        Eigen::MatrixXf K = P * H.transpose() * S.inverse();
        Eigen::VectorXf y = z - H * x;
        y(angle_idx) = wrap_angle(y(angle_idx));

        x += K * y;
        P -= K * H * P;
    }

    double wrap_angle(double theta) {
        return atan2(sin(theta), cos(theta));
    }

private:
    Eigen::VectorXf x;
    Eigen::MatrixXf P;
    int dim;
    int angle_idx;
};



using namespace std::chrono_literals;

class DockNode : public rclcpp::Node {
public:
    DockNode() : Node("dock_node") {
        // target pose tracker
        declare_parameter("target_pose_channel", "exp/target_pose");
        target_pose_ = std::nullopt;
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            get_parameter("target_pose_channel").get_parameter_value().string_value,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
                target_callback(target_pose);
            });

        // CV estimation relay
        state_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("exp/state", 10);
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose2DStamped>(
            "exp/cv_pose",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            [this](const geometry_msgs::msg::Pose2DStamped::SharedPtr cv_pose) {
                est_callback(cv_pose);
            });

        // target state publish loop
        target_timer_ = create_wall_timer(100ms, [this]() {
            target_loop();
        });
        target_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("ctrl/state", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2DStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_pub_;

    std::optional<geometry_msgs::msg::Pose2D> target_pose_;

    void target_loop() {
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
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto dock_node = std::make_shared<DockNode>();
    rclcpp::spin(dock_node);
    rclcpp::shutdown();
    return 0;
}
