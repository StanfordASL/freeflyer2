// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_H_
#define FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dynamics'
// Member 'cost'
// Member 'params_json'
#include "rosidl_runtime_c/string.h"
// Member 'x0'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/PathPlan in the package ff_srvs.
typedef struct ff_srvs__srv__PathPlan_Request
{
  rosidl_runtime_c__String dynamics;
  rosidl_runtime_c__String cost;
  rosidl_runtime_c__double__Sequence x0;
  double t0;
  double dt;
  int64_t horizon;
  int64_t xdim;
  int64_t udim;
  int64_t max_it;
  rosidl_runtime_c__String params_json;
} ff_srvs__srv__PathPlan_Request;

// Struct for a sequence of ff_srvs__srv__PathPlan_Request.
typedef struct ff_srvs__srv__PathPlan_Request__Sequence
{
  ff_srvs__srv__PathPlan_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_srvs__srv__PathPlan_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'times'
// Member 'states'
// Member 'controls'
// Member 'feedback'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/PathPlan in the package ff_srvs.
typedef struct ff_srvs__srv__PathPlan_Response
{
  rosidl_runtime_c__double__Sequence times;
  rosidl_runtime_c__double__Sequence states;
  rosidl_runtime_c__double__Sequence controls;
  rosidl_runtime_c__double__Sequence feedback;
} ff_srvs__srv__PathPlan_Response;

// Struct for a sequence of ff_srvs__srv__PathPlan_Response.
typedef struct ff_srvs__srv__PathPlan_Response__Sequence
{
  ff_srvs__srv__PathPlan_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ff_srvs__srv__PathPlan_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_H_
