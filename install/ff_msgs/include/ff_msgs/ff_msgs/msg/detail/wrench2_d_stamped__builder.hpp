// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/Wrench2DStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/wrench2_d_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_Wrench2DStamped_wrench
{
public:
  explicit Init_Wrench2DStamped_wrench(::ff_msgs::msg::Wrench2DStamped & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::Wrench2DStamped wrench(::ff_msgs::msg::Wrench2DStamped::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::Wrench2DStamped msg_;
};

class Init_Wrench2DStamped_header
{
public:
  Init_Wrench2DStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wrench2DStamped_wrench header(::ff_msgs::msg::Wrench2DStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Wrench2DStamped_wrench(msg_);
  }

private:
  ::ff_msgs::msg::Wrench2DStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::Wrench2DStamped>()
{
  return ff_msgs::msg::builder::Init_Wrench2DStamped_header();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__BUILDER_HPP_
