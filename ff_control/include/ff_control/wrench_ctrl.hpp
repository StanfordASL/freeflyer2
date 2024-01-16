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

#include "ff_control/pwm_ctrl.hpp"

#include "ff_msgs/msg/wrench2_d.hpp"

namespace ff
{

class WrenchController : public PWMController
{
public:
  WrenchController();

protected:
  /**
   * @brief set wrench in body frame
   *
   * @param wrench_body wrench in body frame
   * @param use_wheel   set to true to use the inertial wheel (TODO(alvin): unsupported)
   */
  void SetBodyWrench(const ff_msgs::msg::Wrench2D & wrench_body, bool use_wheel = false);

  /**
   * @brief set wrench in world frame
   *
   * @param wrench_world  wrench in world frame
   * @param theta         rotational state of the freeflyer
   */
  void SetWorldWrench(const ff_msgs::msg::Wrench2D & wrench_world, double theta);

private:
  ff_msgs::msg::Wrench2D ClipWrench(const ff_msgs::msg::Wrench2D & wrench) const;
};

}  // namespace ff
