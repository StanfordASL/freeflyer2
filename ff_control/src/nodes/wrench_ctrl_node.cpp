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


#include <atomic>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/wrench2_d.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

#include "ff_control/wrench_ctrl.hpp"

using namespace std::placeholders;

class WrenchControlNode : public ff::WrenchController
{
public:
  WrenchControlNode()
  : rclcpp::Node("wrench_control_node"),
    ff::WrenchController()
  {
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

  void WrenchBodyCallback(const ff_msgs::msg::Wrench2D::SharedPtr msg)
  {
    SetBodyWrench(*msg);
  }

  void WrenchWorldCallback(const ff_msgs::msg::Wrench2D::SharedPtr msg)
  {
    SetWorldWrench(*msg, pose_.pose.theta);
  }

  void PoseCallback(const ff_msgs::msg::Pose2DStamped::SharedPtr msg)
  {
    pose_ = *msg;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchControlNode>());
  rclcpp::shutdown();
  return 0;
}
