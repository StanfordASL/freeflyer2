// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ff_msgs:msg/FreeFlyerState.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__TRAITS_HPP_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ff_msgs/msg/detail/free_flyer_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "ff_msgs/msg/detail/pose2_d__traits.hpp"
// Member 'twist'
#include "ff_msgs/msg/detail/twist2_d__traits.hpp"

namespace ff_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FreeFlyerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FreeFlyerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FreeFlyerState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ff_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ff_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ff_msgs::msg::FreeFlyerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ff_msgs::msg::FreeFlyerState & msg)
{
  return ff_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ff_msgs::msg::FreeFlyerState>()
{
  return "ff_msgs::msg::FreeFlyerState";
}

template<>
inline const char * name<ff_msgs::msg::FreeFlyerState>()
{
  return "ff_msgs/msg/FreeFlyerState";
}

template<>
struct has_fixed_size<ff_msgs::msg::FreeFlyerState>
  : std::integral_constant<bool, has_fixed_size<ff_msgs::msg::Pose2D>::value && has_fixed_size<ff_msgs::msg::Twist2D>::value> {};

template<>
struct has_bounded_size<ff_msgs::msg::FreeFlyerState>
  : std::integral_constant<bool, has_bounded_size<ff_msgs::msg::Pose2D>::value && has_bounded_size<ff_msgs::msg::Twist2D>::value> {};

template<>
struct is_message<ff_msgs::msg::FreeFlyerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__TRAITS_HPP_
