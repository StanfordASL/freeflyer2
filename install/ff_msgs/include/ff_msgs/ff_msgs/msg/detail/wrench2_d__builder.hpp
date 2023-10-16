// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/Wrench2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/wrench2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_Wrench2D_tz
{
public:
  explicit Init_Wrench2D_tz(::ff_msgs::msg::Wrench2D & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::Wrench2D tz(::ff_msgs::msg::Wrench2D::_tz_type arg)
  {
    msg_.tz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::Wrench2D msg_;
};

class Init_Wrench2D_fy
{
public:
  explicit Init_Wrench2D_fy(::ff_msgs::msg::Wrench2D & msg)
  : msg_(msg)
  {}
  Init_Wrench2D_tz fy(::ff_msgs::msg::Wrench2D::_fy_type arg)
  {
    msg_.fy = std::move(arg);
    return Init_Wrench2D_tz(msg_);
  }

private:
  ::ff_msgs::msg::Wrench2D msg_;
};

class Init_Wrench2D_fx
{
public:
  Init_Wrench2D_fx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wrench2D_fy fx(::ff_msgs::msg::Wrench2D::_fx_type arg)
  {
    msg_.fx = std::move(arg);
    return Init_Wrench2D_fy(msg_);
  }

private:
  ::ff_msgs::msg::Wrench2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::Wrench2D>()
{
  return ff_msgs::msg::builder::Init_Wrench2D_fx();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D__BUILDER_HPP_
