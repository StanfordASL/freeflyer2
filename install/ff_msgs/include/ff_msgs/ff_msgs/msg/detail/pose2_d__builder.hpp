// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__POSE2_D__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__POSE2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/pose2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose2D_theta
{
public:
  explicit Init_Pose2D_theta(::ff_msgs::msg::Pose2D & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::Pose2D theta(::ff_msgs::msg::Pose2D::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::Pose2D msg_;
};

class Init_Pose2D_y
{
public:
  explicit Init_Pose2D_y(::ff_msgs::msg::Pose2D & msg)
  : msg_(msg)
  {}
  Init_Pose2D_theta y(::ff_msgs::msg::Pose2D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose2D_theta(msg_);
  }

private:
  ::ff_msgs::msg::Pose2D msg_;
};

class Init_Pose2D_x
{
public:
  Init_Pose2D_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose2D_y x(::ff_msgs::msg::Pose2D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose2D_y(msg_);
  }

private:
  ::ff_msgs::msg::Pose2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::Pose2D>()
{
  return ff_msgs::msg::builder::Init_Pose2D_x();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__POSE2_D__BUILDER_HPP_
