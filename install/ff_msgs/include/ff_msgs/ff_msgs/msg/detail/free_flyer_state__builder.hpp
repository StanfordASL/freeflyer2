// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/FreeFlyerState.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/free_flyer_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_FreeFlyerState_twist
{
public:
  explicit Init_FreeFlyerState_twist(::ff_msgs::msg::FreeFlyerState & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::FreeFlyerState twist(::ff_msgs::msg::FreeFlyerState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::FreeFlyerState msg_;
};

class Init_FreeFlyerState_pose
{
public:
  Init_FreeFlyerState_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FreeFlyerState_twist pose(::ff_msgs::msg::FreeFlyerState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_FreeFlyerState_twist(msg_);
  }

private:
  ::ff_msgs::msg::FreeFlyerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::FreeFlyerState>()
{
  return ff_msgs::msg::builder::Init_FreeFlyerState_pose();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__BUILDER_HPP_
