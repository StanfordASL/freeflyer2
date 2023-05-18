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

#include "ff_estimate/base_estimator.hpp"
#include "ff_msgs/msg/free_flyer_state_stamped.hpp"
#include "ff_msgs/msg/pose2_d_stamped.hpp"

namespace ff
{

/**
 * @brief full-state estimator using only pose measurements
 */
class Pose2DEstimator : public BaseEstimator
{
public:
  explicit Pose2DEstimator(const std::string & node_name);

protected:
  /**
   * @brief estimate state with Pose2D
   *
   * @param pose_stamped timestamped pose2d
   */
  virtual void EstimateWithPose2D(const ff_msgs::msg::Pose2DStamped & pose_stamped);

private:
  ff_msgs::msg::FreeFlyerStateStamped prev_;
  bool prev_state_ready_ = false;
};

}  // namespace ff
