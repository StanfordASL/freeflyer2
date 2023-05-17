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


#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ff_msgs/msg/free_flyer_state_stamped.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class Renderer : public rclcpp::Node
{
public:
  explicit Renderer(const std::string & name)
  : rclcpp::Node(name)
  {
    auto robot_names = this->declare_parameter<std::vector<std::string>>("robot_name", {"robot"});
    state_subs_.resize(robot_names.size());
    transform_msgs_.resize(robot_names.size());

    // build marker for the table
    marker_msg_.ns = "table";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_msg_.mesh_resource = "package://ff_viz/model/granite_table.stl";
    marker_msg_.scale.x = 0.001;
    marker_msg_.scale.y = 0.001;
    marker_msg_.scale.z = 0.001;
    marker_msg_.header.frame_id = "map";
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.pose.position.x = 0;
    marker_msg_.pose.position.y = 2.7;
    marker_msg_.pose.position.z = -0.9;
    marker_msg_.pose.orientation.x = std::cos(0.25 * M_PI);
    marker_msg_.pose.orientation.y = 0;
    marker_msg_.pose.orientation.z = 0;
    marker_msg_.pose.orientation.w = std::sin(0.25 * M_PI);
    marker_msg_.color.r = 0.5;
    marker_msg_.color.g = 0.5;
    marker_msg_.color.b = 0.5;
    marker_msg_.color.a = 1.0;
    marker_msg_.lifetime = rclcpp::Duration::from_seconds(0);

    for (size_t i = 0; i < robot_names.size(); ++i) {
      // build pose messages
      transform_msgs_[i] = std::make_unique<geometry_msgs::msg::TransformStamped>();
      transform_msgs_[i]->header.frame_id = "map";
      transform_msgs_[i]->child_frame_id = robot_names[i] + "/base";
      transform_msgs_[i]->transform.rotation.x = 0.0;
      transform_msgs_[i]->transform.rotation.y = 0.0;
      transform_msgs_[i]->transform.rotation.z = 0.0;
      transform_msgs_[i]->transform.rotation.w = 1.0;
      transform_msgs_[i]->transform.translation.x = 0.0;
      transform_msgs_[i]->transform.translation.y = 0.0;
      transform_msgs_[i]->transform.translation.z = 0.0;

      // create pose subscription
      state_subs_[i] = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
        "/" + robot_names[i] + "/gt/state",
        10,
        [this, i](const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg) {
          this->StateCallback(i, msg);
        }
      );
    }

    // marker publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("table_marker", 10);

    // tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // periodically send out marker message
    timer_ = this->create_wall_timer(20ms, std::bind(&Renderer::VizUpdate, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr> state_subs_;

  visualization_msgs::msg::Marker marker_msg_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped::UniquePtr> transform_msgs_;

  void VizUpdate()
  {
    // render table
    marker_msg_.header.stamp = this->get_clock()->now();
    marker_pub_->publish(marker_msg_);

    // render robot state
    for (size_t i = 0; i < transform_msgs_.size(); ++i) {
      tf_broadcaster_->sendTransform(*transform_msgs_[i]);
    }
  }

  void StateCallback(const size_t idx, const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg)
  {
    transform_msgs_[idx]->transform.translation.x = msg->state.pose.x;
    transform_msgs_[idx]->transform.translation.y = msg->state.pose.y;
    transform_msgs_[idx]->transform.translation.z = 0.15;

    const double theta = std::remainder(msg->state.pose.theta, 2 * M_PI);
    transform_msgs_[idx]->transform.rotation.x = 0.;
    transform_msgs_[idx]->transform.rotation.y = 0.;
    transform_msgs_[idx]->transform.rotation.z = std::sin(theta / 2);
    transform_msgs_[idx]->transform.rotation.w = std::cos(theta / 2);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Renderer>("renderer_node"));
  rclcpp::shutdown();
  return 0;
}
