// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ff_srvs/srv/detail/path_plan__rosidl_typesupport_introspection_c.h"
#include "ff_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ff_srvs/srv/detail/path_plan__functions.h"
#include "ff_srvs/srv/detail/path_plan__struct.h"


// Include directives for member types
// Member `dynamics`
// Member `cost`
// Member `params_json`
#include "rosidl_runtime_c/string_functions.h"
// Member `x0`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ff_srvs__srv__PathPlan_Request__init(message_memory);
}

void ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_fini_function(void * message_memory)
{
  ff_srvs__srv__PathPlan_Request__fini(message_memory);
}

size_t ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__size_function__PathPlan_Request__x0(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Request__x0(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_function__PathPlan_Request__x0(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Request__x0(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Request__x0(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__assign_function__PathPlan_Request__x0(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_function__PathPlan_Request__x0(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__resize_function__PathPlan_Request__x0(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_member_array[10] = {
  {
    "dynamics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, dynamics),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cost",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, cost),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, x0),  // bytes offset in struct
    NULL,  // default value
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__size_function__PathPlan_Request__x0,  // size() function pointer
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Request__x0,  // get_const(index) function pointer
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__get_function__PathPlan_Request__x0,  // get(index) function pointer
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Request__x0,  // fetch(index, &value) function pointer
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__assign_function__PathPlan_Request__x0,  // assign(index, value) function pointer
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__resize_function__PathPlan_Request__x0  // resize(index) function pointer
  },
  {
    "t0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, t0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, dt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "horizon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, horizon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "xdim",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, xdim),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "udim",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, udim),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_it",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, max_it),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "params_json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Request, params_json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_members = {
  "ff_srvs__srv",  // message namespace
  "PathPlan_Request",  // message name
  10,  // number of fields
  sizeof(ff_srvs__srv__PathPlan_Request),
  ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_member_array,  // message members
  ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_type_support_handle = {
  0,
  &ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ff_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Request)() {
  if (!ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_type_support_handle.typesupport_identifier) {
    ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ff_srvs__srv__PathPlan_Request__rosidl_typesupport_introspection_c__PathPlan_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ff_srvs/srv/detail/path_plan__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ff_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ff_srvs/srv/detail/path_plan__functions.h"
// already included above
// #include "ff_srvs/srv/detail/path_plan__struct.h"


// Include directives for member types
// Member `times`
// Member `states`
// Member `controls`
// Member `feedback`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ff_srvs__srv__PathPlan_Response__init(message_memory);
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_fini_function(void * message_memory)
{
  ff_srvs__srv__PathPlan_Response__fini(message_memory);
}

size_t ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__times(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__times(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__times(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__times(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__times(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__times(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__states(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__states(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__states(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__states(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__states(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__states(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__controls(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__controls(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__controls(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__controls(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__controls(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__controls(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__controls(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__controls(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__feedback(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__feedback(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__feedback(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__feedback(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__feedback(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__feedback(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__feedback(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__feedback(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_member_array[4] = {
  {
    "times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Response, times),  // bytes offset in struct
    NULL,  // default value
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__times,  // size() function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__times,  // get_const(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__times,  // get(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__times,  // fetch(index, &value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__times,  // assign(index, value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__times  // resize(index) function pointer
  },
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Response, states),  // bytes offset in struct
    NULL,  // default value
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__states,  // size() function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__states,  // get_const(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__states,  // get(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__states,  // fetch(index, &value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__states,  // assign(index, value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__states  // resize(index) function pointer
  },
  {
    "controls",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Response, controls),  // bytes offset in struct
    NULL,  // default value
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__controls,  // size() function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__controls,  // get_const(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__controls,  // get(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__controls,  // fetch(index, &value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__controls,  // assign(index, value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__controls  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_srvs__srv__PathPlan_Response, feedback),  // bytes offset in struct
    NULL,  // default value
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__size_function__PathPlan_Response__feedback,  // size() function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_const_function__PathPlan_Response__feedback,  // get_const(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__get_function__PathPlan_Response__feedback,  // get(index) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__fetch_function__PathPlan_Response__feedback,  // fetch(index, &value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__assign_function__PathPlan_Response__feedback,  // assign(index, value) function pointer
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__resize_function__PathPlan_Response__feedback  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_members = {
  "ff_srvs__srv",  // message namespace
  "PathPlan_Response",  // message name
  4,  // number of fields
  sizeof(ff_srvs__srv__PathPlan_Response),
  ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_member_array,  // message members
  ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_type_support_handle = {
  0,
  &ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ff_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Response)() {
  if (!ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_type_support_handle.typesupport_identifier) {
    ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ff_srvs__srv__PathPlan_Response__rosidl_typesupport_introspection_c__PathPlan_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ff_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ff_srvs/srv/detail/path_plan__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_members = {
  "ff_srvs__srv",  // service namespace
  "PathPlan",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_Request_message_type_support_handle,
  NULL  // response message
  // ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_Response_message_type_support_handle
};

static rosidl_service_type_support_t ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_type_support_handle = {
  0,
  &ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ff_srvs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan)() {
  if (!ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_type_support_handle.typesupport_identifier) {
    ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ff_srvs, srv, PathPlan_Response)()->data;
  }

  return &ff_srvs__srv__detail__path_plan__rosidl_typesupport_introspection_c__PathPlan_service_type_support_handle;
}
