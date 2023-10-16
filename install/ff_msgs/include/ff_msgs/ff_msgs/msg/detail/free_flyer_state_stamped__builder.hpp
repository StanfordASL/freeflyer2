// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_msgs:msg/FreeFlyerStateStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__BUILDER_HPP_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_msgs/msg/detail/free_flyer_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_msgs
{

namespace msg
{

namespace builder
{

class Init_FreeFlyerStateStamped_state
{
public:
  explicit Init_FreeFlyerStateStamped_state(::ff_msgs::msg::FreeFlyerStateStamped & msg)
  : msg_(msg)
  {}
  ::ff_msgs::msg::FreeFlyerStateStamped state(::ff_msgs::msg::FreeFlyerStateStamped::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_msgs::msg::FreeFlyerStateStamped msg_;
};

class Init_FreeFlyerStateStamped_header
{
public:
  Init_FreeFlyerStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FreeFlyerStateStamped_state header(::ff_msgs::msg::FreeFlyerStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FreeFlyerStateStamped_state(msg_);
  }

private:
  ::ff_msgs::msg::FreeFlyerStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_msgs::msg::FreeFlyerStateStamped>()
{
  return ff_msgs::msg::builder::Init_FreeFlyerStateStamped_header();
}

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__BUILDER_HPP_
