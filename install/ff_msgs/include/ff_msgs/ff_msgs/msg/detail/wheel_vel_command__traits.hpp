// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ff_msgs:msg/WheelVelCommand.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__TRAITS_HPP_
#define FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ff_msgs/msg/detail/wheel_vel_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ff_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WheelVelCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WheelVelCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WheelVelCommand & msg, bool use_flow_style = false)
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
  const ff_msgs::msg::WheelVelCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ff_msgs::msg::WheelVelCommand & msg)
{
  return ff_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ff_msgs::msg::WheelVelCommand>()
{
  return "ff_msgs::msg::WheelVelCommand";
}

template<>
inline const char * name<ff_msgs::msg::WheelVelCommand>()
{
  return "ff_msgs/msg/WheelVelCommand";
}

template<>
struct has_fixed_size<ff_msgs::msg::WheelVelCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ff_msgs::msg::WheelVelCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ff_msgs::msg::WheelVelCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__TRAITS_HPP_
