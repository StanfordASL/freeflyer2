// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/Twist2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__TWIST2_D__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__TWIST2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/twist2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_Twist2D_wz
{
public:
  explicit Init_Twist2D_wz(::ff_msgs::msg::Twist2D & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::Twist2D wz(::ff_msgs::msg::Twist2D::_wz_type arg)
  {
    msg_.wz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::Twist2D msg_;
};

class Init_Twist2D_vy
{
public:
  explicit Init_Twist2D_vy(::ff_msgs::msg::Twist2D & msg)
  : msg_(msg)
  {}
  Init_Twist2D_wz vy(::ff_msgs::msg::Twist2D::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_Twist2D_wz(msg_);
  }

private:
  ::ff_msgs::msg::Twist2D msg_;
};

class Init_Twist2D_vx
{
public:
  Init_Twist2D_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Twist2D_vy vx(::ff_msgs::msg::Twist2D::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_Twist2D_vy(msg_);
  }

private:
  ::ff_msgs::msg::Twist2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::Twist2D>()
{
  return ff_msgs::msg::builder::Init_Twist2D_vx();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__TWIST2_D__BUILDER_HPP_
