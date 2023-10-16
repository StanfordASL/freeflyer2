// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ff_msgs:msg/Twist2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__TWIST2_D__TRAITS_HPP_
#define FF_MSGS__MSG__DETAIL__TWIST2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ff_msgs/msg/detail/twist2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ff_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Twist2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: wz
  {
    out << "wz: ";
    rosidl_generator_traits::value_to_yaml(msg.wz, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Twist2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: wz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wz: ";
    rosidl_generator_traits::value_to_yaml(msg.wz, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Twist2D & msg, bool use_flow_style = false)
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
  const ff_msgs::msg::Twist2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ff_msgs::msg::Twist2D & msg)
{
  return ff_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ff_msgs::msg::Twist2D>()
{
  return "ff_msgs::msg::Twist2D";
}

template<>
inline const char * name<ff_msgs::msg::Twist2D>()
{
  return "ff_msgs/msg/Twist2D";
}

template<>
struct has_fixed_size<ff_msgs::msg::Twist2D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ff_msgs::msg::Twist2D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ff_msgs::msg::Twist2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FF_MSGS__MSG__DETAIL__TWIST2_D__TRAITS_HPP_
