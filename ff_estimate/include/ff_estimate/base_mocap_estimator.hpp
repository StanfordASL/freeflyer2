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


#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace ff
{

/**
 * @brief estimator base class
 */
class BaseMocapEstimator : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   *
   * @param node_name ROS2 node name, default to mocap_estimator
   */
  explicit BaseMocapEstimator(const std::string & node_name = "mocap_estimator");

protected:
  /**
   * @brief send out the current state estimate
   *
   * @param state estimated freeflyer state
   */
  void SendStateEstimate(const ff_msgs::msg::FreeFlyerState & state);

  /**
   * @brief abstract estimator function
   *
   * @param pose_stamped time-stamped 2D pose measurement
   */
  virtual void EstimateWithPose2D(const ff_msgs::msg::Pose2DStamped & pose_stamped) = 0;

private:
  rclcpp::Publisher<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
};

}  // namespace ff
