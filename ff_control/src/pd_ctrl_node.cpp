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

#include "ff_control/wrench_ctrl.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class PDControlNode : public ff::WrenchController {
 public:
  PDControlNode()
    : ff::WrenchController("pd_control_node"),
      pose_{}, twist_{}, pose_des_{},
      gain_f(declare_parameter("gain_f", 2.0)),
      gain_df(declare_parameter("gain_df", 10.0)),
      gain_t(declare_parameter("gain_t", 0.2)),
      gain_dt(declare_parameter("gain_dt", 0.4)) {
    pose_setpoint_sub_ = this->create_subscription<ff_msgs::msg::Pose2D>(
      "ctrl/pose", 10, std::bind(&PDControlNode::SetpointCallback, this, _1));
    rviz_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PDControlNode::GoalPoseCallback, this, _1));
    pose_sub_ = this->create_subscription<ff_msgs::msg::Pose2DStamped>(
      "gt/pose", 10, std::bind(&PDControlNode::PoseCallback, this, _1));
    twist_sub_ = this->create_subscription<ff_msgs::msg::Twist2DStamped>(
      "gt/twist", 10, std::bind(&PDControlNode::TwistCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&PDControlNode::ControlLoop, this));


  }

 private:
  rclcpp::Subscription<ff_msgs::msg::Pose2D>::SharedPtr pose_setpoint_sub_;
  rclcpp::Subscription<ff_msgs::msg::Pose2DStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<ff_msgs::msg::Twist2DStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ff_msgs::msg::Pose2DStamped pose_;
  ff_msgs::msg::Twist2DStamped twist_;
  ff_msgs::msg::Pose2D pose_des_;

  const double gain_f;
  const double gain_df;
  const double gain_t;
  const double gain_dt;

  void ControlLoop() {
    ff_msgs::msg::Wrench2D wrench_world;
    const double angle_diff = std::remainder(pose_des_.theta - pose_.pose.theta, 2 * M_PI);
    wrench_world.fx = gain_f * (pose_des_.x - pose_.pose.x) + gain_df * (-twist_.twist.vx);
    wrench_world.fy = gain_f * (pose_des_.y - pose_.pose.y) + gain_df * (-twist_.twist.vy);
    wrench_world.tz = gain_t * angle_diff + gain_dt * (-twist_.twist.wz);

    SetWorldWrench(wrench_world, pose_.pose.theta);
  }

  void SetpointCallback(const ff_msgs::msg::Pose2D::SharedPtr msg) {
    pose_des_ = *msg;
  }

  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_des_.x = msg->pose.position.x;
    pose_des_.y = msg->pose.position.y;
    pose_des_.theta = 2 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  }

  void PoseCallback(const ff_msgs::msg::Pose2DStamped::SharedPtr msg) {
    pose_ = *msg;
  }

  void TwistCallback(const ff_msgs::msg::Twist2DStamped::SharedPtr msg) {
    twist_ = *msg;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PDControlNode>());
  rclcpp::shutdown();
  return 0;
}
