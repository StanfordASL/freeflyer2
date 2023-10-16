// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_msgs:msg/Twist2DStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__TWIST2_D_STAMPED__STRUCT_H_
#define FF_MSGS__MSG__DETAIL__TWIST2_D_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'twist'
#include "ff_msgs/msg/detail/twist2_d__struct.h"

/// Struct defined in msg/Twist2DStamped in the package ff_msgs.
typedef struct ff_msgs__msg__Twist2DStamped
{
  std_msgs__msg__Header header;
  ff_msgs__msg__Twist2D twist;
} ff_msgs__msg__Twist2DStamped;

// Struct for a sequence of ff_msgs__msg__Twist2DStamped.
typedef struct ff_msgs__msg__Twist2DStamped__Sequence
{
  ff_msgs__msg__Twist2DStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_msgs__msg__Twist2DStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__TWIST2_D_STAMPED__STRUCT_H_
