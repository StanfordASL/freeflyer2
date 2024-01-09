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

using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::FreeFlyerStateStamped;
using ff_msgs::msg::Pose2DStamped;
using geometry_msgs::msg::PoseStamped;

namespace ff
{

BaseMocapEstimator::BaseMocapEstimator(const std::string & node_name)
: rclcpp::Node(node_name)
{
  const std::string pose_channel = this->declare_parameter("pose_channel", "mocap/sim/pose");
  pose_sub_ = this->create_subscription<PoseStamped>(
    pose_channel, rclcpp::SensorDataQoS(), [this](const PoseStamped::SharedPtr msg) {
      PoseCallback(msg);
    });
  state_pub_ = this->create_publisher<FreeFlyerStateStamped>("est/state", 10);
}

void BaseMocapEstimator::SendStateEstimate(const FreeFlyerState & state)
{
  ff_msgs::msg::FreeFlyerStateStamped msg{};
  msg.state = state;
  msg.header.stamp = this->get_clock()->now();

  state_pub_->publish(msg);
}

void BaseMocapEstimator::PoseCallback(const PoseStamped::SharedPtr pose)
{
  // convert to 3D pose to 2D pose
  Pose2DStamped pose2d{};

  pose2d.header = pose->header;
  pose2d.pose.x = pose->pose.position.x;
  pose2d.pose.y = pose->pose.position.y;
  double w = pose->pose.orientation.w;
  double z = pose->pose.orientation.z;
  pose2d.pose.theta = std::atan2(2 * w * z, w * w - z * z);

  EstimateWithPose2D(pose2d);
}

}  // namespace ff
