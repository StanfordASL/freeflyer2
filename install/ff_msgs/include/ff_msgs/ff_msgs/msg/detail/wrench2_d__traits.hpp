// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ff_msgs:msg/Wrench2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D__TRAITS_HPP_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ff_msgs/msg/detail/wrench2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ff_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Wrench2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: fx
  {
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << ", ";
  }

  // member: fy
  {
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << ", ";
  }

  // member: tz
  {
    out << "tz: ";
    rosidl_generator_traits::value_to_yaml(msg.tz, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wrench2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << "\n";
  }

  // member: fy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << "\n";
  }

  // member: tz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tz: ";
    rosidl_generator_traits::value_to_yaml(msg.tz, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wrench2D & msg, bool use_flow_style = false)
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
  const ff_msgs::msg::Wrench2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ff_msgs::msg::Wrench2D & msg)
{
  return ff_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ff_msgs::msg::Wrench2D>()
{
  return "ff_msgs::msg::Wrench2D";
}

template<>
inline const char * name<ff_msgs::msg::Wrench2D>()
{
  return "ff_msgs/msg/Wrench2D";
}

template<>
struct has_fixed_size<ff_msgs::msg::Wrench2D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ff_msgs::msg::Wrench2D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ff_msgs::msg::Wrench2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D__TRAITS_HPP_
