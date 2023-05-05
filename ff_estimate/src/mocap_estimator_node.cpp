#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ff_estimate/pose2d_estimator.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

using ff_msgs::msg::Pose2DStamped;
using geometry_msgs::msg::PoseStamped;

class MocapEstimatorNode : public ff::Pose2DEstimator {
 public:
  MocapEstimatorNode() : ff::Pose2DEstimator("mocap_estimator_node") {
    const std::string pose_channel = this->declare_parameter("pose_channel", "sim/mocap/pose");
    pose_sub_ = this->create_subscription<PoseStamped>(
      pose_channel, 10, [this](const PoseStamped::SharedPtr msg) {PoseCallback(msg);});
  }

 private:
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;

  void PoseCallback(const PoseStamped::SharedPtr pose) {
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapEstimatorNode>());
  rclcpp::shutdown();
}
