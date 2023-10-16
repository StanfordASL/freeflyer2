// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/thruster_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_ThrusterCommand_duty_cycle
{
public:
  explicit Init_ThrusterCommand_duty_cycle(::ff_msgs::msg::ThrusterCommand & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::ThrusterCommand duty_cycle(::ff_msgs::msg::ThrusterCommand::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::ThrusterCommand msg_;
};

class Init_ThrusterCommand_header
{
public:
  Init_ThrusterCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterCommand_duty_cycle header(::ff_msgs::msg::ThrusterCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ThrusterCommand_duty_cycle(msg_);
  }

private:
  ::ff_msgs::msg::ThrusterCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::ThrusterCommand>()
{
  return ff_msgs::msg::builder::Init_ThrusterCommand_header();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_
