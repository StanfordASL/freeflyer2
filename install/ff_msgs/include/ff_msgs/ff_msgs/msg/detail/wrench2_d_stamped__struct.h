// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_msgs:msg/Wrench2DStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__STRUCT_H_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__STRUCT_H_

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
// Member 'wrench'
#include "ff_msgs/msg/detail/wrench2_d__struct.h"

/// Struct defined in msg/Wrench2DStamped in the package ff_msgs.
typedef struct ff_msgs__msg__Wrench2DStamped
{
  std_msgs__msg__Header header;
  ff_msgs__msg__Wrench2D wrench;
} ff_msgs__msg__Wrench2DStamped;

// Struct for a sequence of ff_msgs__msg__Wrench2DStamped.
typedef struct ff_msgs__msg__Wrench2DStamped__Sequence
{
  ff_msgs__msg__Wrench2DStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_msgs__msg__Wrench2DStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D_STAMPED__STRUCT_H_
