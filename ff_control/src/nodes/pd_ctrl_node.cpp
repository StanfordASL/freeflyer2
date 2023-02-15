#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ff_msgs/msg/wrench2_d.hpp"
#include "ff_msgs/msg/pose2_d.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"
#include "ff_msgs/msg/twist2_d_stamped.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

#include "ff_control/wrench_ctrl.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class PDControlNode : public ff::WrenchController {
 public:
  PDControlNode()
    : rclcpp::Node("pd_control_node"),
      ff::WrenchController(),
      state_cur_{}, pose_des_{},
      gain_f(declare_parameter("gain_f", 2.0)),
      gain_df(declare_parameter("gain_df", 10.0)),
      gain_t(declare_parameter("gain_t", 0.2)),
      gain_dt(declare_parameter("gain_dt", 0.4)) {
    pose_setpoint_sub_ = this->create_subscription<ff_msgs::msg::Pose2D>(
      "ctrl/pose", 10, std::bind(&PDControlNode::SetpointCallback, this, _1));
    rviz_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PDControlNode::GoalPoseCallback, this, _1));
    state_sub_ = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
      "gt/state", 10, std::bind(&PDControlNode::StateCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&PDControlNode::ControlLoop, this));


  }

 private:
  rclcpp::Subscription<ff_msgs::msg::Pose2D>::SharedPtr pose_setpoint_sub_;
  rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ff_msgs::msg::FreeFlyerStateStamped state_cur_;
  ff_msgs::msg::Pose2D pose_des_;

  const double gain_f;
  const double gain_df;
  const double gain_t;
  const double gain_dt;

  void ControlLoop() {
    ff_msgs::msg::Wrench2D wrench_world;
    double angle_diff = std::remainder(pose_des_.theta - state_cur_.state.pose.theta, 2*M_PI);
    wrench_world.fx = gain_f * (pose_des_.x - state_cur_.state.pose.x)
                    + gain_df * (-state_cur_.state.twist.vx);
    wrench_world.fy = gain_f * (pose_des_.y - state_cur_.state.pose.y)
                    + gain_df * (-state_cur_.state.twist.vy);
    wrench_world.tz = gain_t * angle_diff + gain_dt * (-state_cur_.state.twist.wz);

    SetWorldWrench(wrench_world, state_cur_.state.pose.theta);
  }

  void SetpointCallback(const ff_msgs::msg::Pose2D::SharedPtr msg) {
    pose_des_ = *msg;
  }

  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_des_.x = msg->pose.position.x;
    pose_des_.y = msg->pose.position.y;
    pose_des_.theta = 2 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  }

  void StateCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg) {
    state_cur_ = *msg;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PDControlNode>());
  rclcpp::shutdown();
  return 0;
}
