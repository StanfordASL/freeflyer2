#include <atomic>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/wrench2_d.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

#include "ff_control/wrench_ctrl.hpp"

using namespace std::placeholders;

class WrenchControlNode : public ff::WrenchController {
 public:
  WrenchControlNode(const std::string& node_name) : ff::WrenchController(node_name) {
    wrench_body_sub_ = this->create_subscription<ff_msgs::msg::Wrench2D>(
      "ctrl/wrench_body", 10, std::bind(&WrenchControlNode::WrenchBodyCallback, this, _1));
    wrench_world_sub_ = this->create_subscription<ff_msgs::msg::Wrench2D>(
      "ctrl/wrench_world", 10, std::bind(&WrenchControlNode::WrenchWorldCallback, this, _1));
    pose_sub_ = this->create_subscription<ff_msgs::msg::Pose2DStamped>(
      "gt/pose", 10, std::bind(&WrenchControlNode::PoseCallback, this, _1));
  }

 private:
  rclcpp::Subscription<ff_msgs::msg::Wrench2D>::SharedPtr wrench_body_sub_;
  rclcpp::Subscription<ff_msgs::msg::Wrench2D>::SharedPtr wrench_world_sub_;
  rclcpp::Subscription<ff_msgs::msg::Pose2DStamped>::SharedPtr pose_sub_;

  ff_msgs::msg::Pose2DStamped pose_;

  void WrenchBodyCallback(const ff_msgs::msg::Wrench2D::SharedPtr msg) {
    SetBodyWrench(*msg);
  }

  void WrenchWorldCallback(const ff_msgs::msg::Wrench2D::SharedPtr msg) {
    SetWorldWrench(*msg, pose_.pose.theta);
  }

  void PoseCallback(const ff_msgs::msg::Pose2DStamped::SharedPtr msg) {
    pose_ = *msg;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchControlNode>("wrench_ctrl_node"));
  rclcpp::shutdown();
  return 0;
}
