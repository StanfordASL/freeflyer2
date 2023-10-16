// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ff_msgs:msg/Wrench2D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ff_msgs/msg/detail/wrench2_d__rosidl_typesupport_introspection_c.h"
#include "ff_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ff_msgs/msg/detail/wrench2_d__functions.h"
#include "ff_msgs/msg/detail/wrench2_d__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ff_msgs__msg__Wrench2D__init(message_memory);
}

void ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_fini_function(void * message_memory)
{
  ff_msgs__msg__Wrench2D__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_member_array[3] = {
  {
    "fx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_msgs__msg__Wrench2D, fx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_msgs__msg__Wrench2D, fy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_msgs__msg__Wrench2D, tz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_members = {
  "ff_msgs__msg",  // message namespace
  "Wrench2D",  // message name
  3,  // number of fields
  sizeof(ff_msgs__msg__Wrench2D),
  ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_member_array,  // message members
  ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_init_function,  // function to initialize message memory (memory has to be allocated)
  ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_type_support_handle = {
  0,
  &ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ff_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_msgs, msg, Wrench2D)() {
  if (!ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_type_support_handle.typesupport_identifier) {
    ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ff_msgs__msg__Wrench2D__rosidl_typesupport_introspection_c__Wrench2D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
