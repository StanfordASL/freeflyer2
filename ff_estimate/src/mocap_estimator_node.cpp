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


#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ff_estimate/pose2d_estimator.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

using ff_msgs::msg::Pose2DStamped;
using geometry_msgs::msg::PoseStamped;

class MocapEstimatorNode : public ff::Pose2DEstimator
{
public:
  MocapEstimatorNode()
  : ff::Pose2DEstimator("mocap_estimator_node")
  {
    const std::string pose_channel = this->declare_parameter("pose_channel", "mocap/sim/pose");
    pose_sub_ = this->create_subscription<PoseStamped>(
      pose_channel, rclcpp::SensorDataQoS(), [this](const PoseStamped::SharedPtr msg) {
        PoseCallback(msg);
      });
  }

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;

  void PoseCallback(const PoseStamped::SharedPtr pose)
  {
    // convert to pose2d
    Pose2DStamped pose2d{};

    pose2d.header = pose->header;
    pose2d.pose.x = pose->pose.position.x;
    pose2d.pose.y = pose->pose.position.y;
    double w = pose->pose.orientation.w;
    double z = pose->pose.orientation.z;
    pose2d.pose.theta = std::atan2(2 * w * z, w * w - z * z);

    this->EstimateWithPose2D(pose2d);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapEstimatorNode>());
  rclcpp::shutdown();
}
