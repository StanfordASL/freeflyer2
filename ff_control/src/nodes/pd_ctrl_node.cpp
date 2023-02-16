#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ff_msgs/msg/pose2_d.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

#include "ff_control/linear_ctrl.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class PDControlNode : public ff::LinearController {
 public:
  PDControlNode()
    : rclcpp::Node("pd_control_node"),
      ff::LinearController(),
      state_des_{},
      K_(FeedbackMat::Zero()) {
    state_setpoint_sub_ = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
      "ctrl/state", 10, std::bind(&PDControlNode::SetpointCallback, this, _1));
    rviz_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PDControlNode::GoalPoseCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&PDControlNode::ControlLoop, this));

    const double gain_f = declare_parameter("gain_f", 2.0);
    const double gain_df = declare_parameter("gain_df", 10.0);
    const double gain_t = declare_parameter("gain_t", 0.2);
    const double gain_dt = declare_parameter("gain_dt", 0.4);

    // construct feedback control matrix
    K_(0, 0) = gain_f;
    K_(0, 3) = gain_df;
    K_(1, 1) = gain_f;
    K_(1, 4) = gain_df;
    K_(2, 2) = gain_t;
    K_(2, 5) = gain_dt;
  }

 private:
  rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_setpoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ff_msgs::msg::FreeFlyerStateStamped state_des_;

  FeedbackMat K_;

  void StateReadyCallback() override {
    // copy current position as goal position
    state_des_.header.stamp = this->get_clock()->now();
    GetState(&state_des_.state);
  }

  void ControlLoop() {
    // state not yet ready
    if (!StateIsReady()) { return; }

    SendControl(state_des_.state, K_);
  }

  void SetpointCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg) {
    state_des_ = *msg;
  }

  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    state_des_.state.pose.x = msg->pose.position.x;
    state_des_.state.pose.y = msg->pose.position.y;

    const double z = msg->pose.orientation.z;
    const double w = msg->pose.orientation.w;
    state_des_.state.pose.theta = std::atan2(2 * w * z, w * w - z * z);

    state_des_.state.twist.vx = 0;
    state_des_.state.twist.vy = 0;
    state_des_.state.twist.wz = 0;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PDControlNode>());
  rclcpp::shutdown();
  return 0;
}
