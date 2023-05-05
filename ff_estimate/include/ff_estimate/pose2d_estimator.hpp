#pragma once

#include "ff_estimate/base_estimator.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

namespace ff {

class Pose2DEstimator: public BaseEstimator {
 public:
  Pose2DEstimator(const std::string& node_name);

 protected:
  void EstimateWithPose2D(const ff_msgs::msg::Pose2DStamped& pose_stamped);

 private:
  ff_msgs::msg::FreeFlyerStateStamped prev_;
  bool prev_state_ready_ = false;
};

} // namespace ff
