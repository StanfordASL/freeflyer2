// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_msgs:msg/FreeFlyerState.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_H_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "ff_msgs/msg/detail/pose2_d__struct.h"
// Member 'twist'
#include "ff_msgs/msg/detail/twist2_d__struct.h"

/// Struct defined in msg/FreeFlyerState in the package ff_msgs.
typedef struct ff_msgs__msg__FreeFlyerState
{
  ff_msgs__msg__Pose2D pose;
  ff_msgs__msg__Twist2D twist;
} ff_msgs__msg__FreeFlyerState;

// Struct for a sequence of ff_msgs__msg__FreeFlyerState.
typedef struct ff_msgs__msg__FreeFlyerState__Sequence
{
  ff_msgs__msg__FreeFlyerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_msgs__msg__FreeFlyerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_H_
