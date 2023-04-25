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
      state_des_{} {
    state_setpoint_sub_ = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
      "ctrl/state", 10, std::bind(&PDControlNode::SetpointCallback, this, _1));
    rviz_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PDControlNode::GoalPoseCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&PDControlNode::ControlLoop, this));

    // default params for feedback gain matrix
    this->declare_parameter("gain_f", 2.0);
    this->declare_parameter("gain_df", 10.0);
    this->declare_parameter("gain_t", 0.2);
    this->declare_parameter("gain_dt", 0.4);
  }

 private:
  rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_setpoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ff_msgs::msg::FreeFlyerStateStamped state_des_;

  void StateReadyCallback() override {
    // copy current position as goal position
    state_des_.header.stamp = this->get_clock()->now();
    GetState(&state_des_.state);
  }

  void ControlLoop() {
    // state not yet ready
    if (!StateIsReady()) { return; }

    // build feedback control matrix
    const double gain_f = this->get_parameter("gain_f").as_double();
    const double gain_df = this->get_parameter("gain_df").as_double();
    const double gain_t = this->get_parameter("gain_t").as_double();
    const double gain_dt = this->get_parameter("gain_dt").as_double();
    FeedbackMat K = FeedbackMat::Zero();
    K(0, 0) = gain_f;
    K(0, 3) = gain_df;
    K(1, 1) = gain_f;
    K(1, 4) = gain_df;
    K(2, 2) = gain_t;
    K(2, 5) = gain_dt;

    SendControl(state_des_.state, K);
  }

  void SetpointCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg) {
    state_des_ = *msg;
  }

  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    state_des_.header.stamp = msg->header.stamp;

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
