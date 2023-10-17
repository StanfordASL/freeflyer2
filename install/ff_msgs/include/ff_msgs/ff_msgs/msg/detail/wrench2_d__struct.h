// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_msgs:msg/Wrench2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_H_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Wrench2D in the package ff_msgs.
typedef struct ff_msgs__msg__Wrench2D
{
  double fx;
  double fy;
  double tz;
} ff_msgs__msg__Wrench2D;

// Struct for a sequence of ff_msgs__msg__Wrench2D.
typedef struct ff_msgs__msg__Wrench2D__Sequence
{
  ff_msgs__msg__Wrench2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_msgs__msg__Wrench2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_H_