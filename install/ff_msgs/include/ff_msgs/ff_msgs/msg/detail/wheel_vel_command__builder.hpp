// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/WheelVelCommand.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/wheel_vel_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_WheelVelCommand_velocity
{
public:
  explicit Init_WheelVelCommand_velocity(::ff_msgs::msg::WheelVelCommand & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::WheelVelCommand velocity(::ff_msgs::msg::WheelVelCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::WheelVelCommand msg_;
};

class Init_WheelVelCommand_header
{
public:
  Init_WheelVelCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelVelCommand_velocity header(::ff_msgs::msg::WheelVelCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WheelVelCommand_velocity(msg_);
  }

private:
  ::ff_msgs::msg::WheelVelCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::WheelVelCommand>()
{
  return ff_msgs::msg::builder::Init_WheelVelCommand_header();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__BUILDER_HPP_
