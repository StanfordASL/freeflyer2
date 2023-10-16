// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_msgs:msg/FreeFlyerStateStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_H_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_H_

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
// Member 'state'
#include "ff_msgs/msg/detail/free_flyer_state__struct.h"

/// Struct defined in msg/FreeFlyerStateStamped in the package ff_msgs.
typedef struct ff_msgs__msg__FreeFlyerStateStamped
{
  std_msgs__msg__Header header;
  ff_msgs__msg__FreeFlyerState state;
} ff_msgs__msg__FreeFlyerStateStamped;

// Struct for a sequence of ff_msgs__msg__FreeFlyerStateStamped.
typedef struct ff_msgs__msg__FreeFlyerStateStamped__Sequence
{
  ff_msgs__msg__FreeFlyerStateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_msgs__msg__FreeFlyerStateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_H_
