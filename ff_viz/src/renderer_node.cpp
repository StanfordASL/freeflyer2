#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "ff_msgs/msg/pose2_d_stamped.hpp"

using namespace std::chrono_literals;

class TableRenderer : public rclcpp::Node {
 public:
  TableRenderer(const std::string& name) : rclcpp::Node(name) {
    auto robot_names = this->declare_parameter<std::vector<std::string>>("robot_name", {"robot"});
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

    // build pose msg
    transform_msg_.header.frame_id = "map";
    transform_msg_.child_frame_id = robot_names[0] + "/base";
    transform_msg_.transform.rotation.x = 0.0;
    transform_msg_.transform.rotation.y = 0.0;
    transform_msg_.transform.rotation.z = 0.0;
    transform_msg_.transform.rotation.w = 1.0;
    transform_msg_.transform.translation.x = 0.0;
    transform_msg_.transform.translation.y = 0.0;
    transform_msg_.transform.translation.z = 0.0;

    pose_sub_ = this->create_subscription<ff_msgs::msg::Pose2DStamped>(
      "/robot/estimator/pose",
      10,
      std::bind(&TableRenderer::PoseCallback, this, std::placeholders::_1)
    );

    // marker publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("table_marker", 10);

    // tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // periodically send out marker message
    timer_ = this->create_wall_timer(20ms, std::bind(&TableRenderer::VizUpdate, this));
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<ff_msgs::msg::Pose2DStamped>::SharedPtr pose_sub_;

  visualization_msgs::msg::Marker marker_msg_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_msg_;

  void VizUpdate() {
    // render table
    marker_msg_.header.stamp = this->get_clock()->now();
    marker_pub_->publish(marker_msg_);

    // render robot state
    tf_broadcaster_->sendTransform(transform_msg_);
  }

  void PoseCallback(const ff_msgs::msg::Pose2DStamped::SharedPtr msg) {
    transform_msg_.transform.translation.x = msg->x;
    transform_msg_.transform.translation.y = msg->y;
    transform_msg_.transform.translation.z = 0.15;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform_msg_.transform.rotation.x = q.x();
    transform_msg_.transform.rotation.y = q.y();
    transform_msg_.transform.rotation.z = q.z();
    transform_msg_.transform.rotation.w = q.w();
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TableRenderer>("renderer_node"));
  rclcpp::shutdown();
  return 0;
}
