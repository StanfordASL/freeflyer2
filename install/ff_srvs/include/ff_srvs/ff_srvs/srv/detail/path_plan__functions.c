// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice
#include "ff_srvs/srv/detail/path_plan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `dynamics`
// Member `cost`
// Member `params_json`
#include "rosidl_runtime_c/string_functions.h"
// Member `x0`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ff_srvs__srv__PathPlan_Request__init(ff_srvs__srv__PathPlan_Request * msg)
{
  if (!msg) {
    return false;
  }
  // dynamics
  if (!rosidl_runtime_c__String__init(&msg->dynamics)) {
    ff_srvs__srv__PathPlan_Request__fini(msg);
    return false;
  }
  // cost
  if (!rosidl_runtime_c__String__init(&msg->cost)) {
    ff_srvs__srv__PathPlan_Request__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->cost, "default_cost");
    if (!success) {
      goto abort_init_0;
    }
  }
  // x0
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x0, 0)) {
    ff_srvs__srv__PathPlan_Request__fini(msg);
    return false;
  }
  // t0
  // dt
  msg->dt = 0.25l;
  // horizon
  msg->horizon = 20ll;
  // xdim
  msg->xdim = -1ll;
  // udim
  msg->udim = -1ll;
  // max_it
  msg->max_it = 20ll;
  // params_json
  if (!rosidl_runtime_c__String__init(&msg->params_json)) {
    ff_srvs__srv__PathPlan_Request__fini(msg);
    return false;
  }
  return true;
abort_init_0:
  return false;
}

void
ff_srvs__srv__PathPlan_Request__fini(ff_srvs__srv__PathPlan_Request * msg)
{
  if (!msg) {
    return;
  }
  // dynamics
  rosidl_runtime_c__String__fini(&msg->dynamics);
  // cost
  rosidl_runtime_c__String__fini(&msg->cost);
  // x0
  rosidl_runtime_c__double__Sequence__fini(&msg->x0);
  // t0
  // dt
  // horizon
  // xdim
  // udim
  // max_it
  // params_json
  rosidl_runtime_c__String__fini(&msg->params_json);
}

bool
ff_srvs__srv__PathPlan_Request__are_equal(const ff_srvs__srv__PathPlan_Request * lhs, const ff_srvs__srv__PathPlan_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dynamics
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dynamics), &(rhs->dynamics)))
  {
    return false;
  }
  // cost
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cost), &(rhs->cost)))
  {
    return false;
  }
  // x0
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x0), &(rhs->x0)))
  {
    return false;
  }
  // t0
  if (lhs->t0 != rhs->t0) {
    return false;
  }
  // dt
  if (lhs->dt != rhs->dt) {
    return false;
  }
  // horizon
  if (lhs->horizon != rhs->horizon) {
    return false;
  }
  // xdim
  if (lhs->xdim != rhs->xdim) {
    return false;
  }
  // udim
  if (lhs->udim != rhs->udim) {
    return false;
  }
  // max_it
  if (lhs->max_it != rhs->max_it) {
    return false;
  }
  // params_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->params_json), &(rhs->params_json)))
  {
    return false;
  }
  return true;
}

bool
ff_srvs__srv__PathPlan_Request__copy(
  const ff_srvs__srv__PathPlan_Request * input,
  ff_srvs__srv__PathPlan_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // dynamics
  if (!rosidl_runtime_c__String__copy(
      &(input->dynamics), &(output->dynamics)))
  {
    return false;
  }
  // cost
  if (!rosidl_runtime_c__String__copy(
      &(input->cost), &(output->cost)))
  {
    return false;
  }
  // x0
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x0), &(output->x0)))
  {
    return false;
  }
  // t0
  output->t0 = input->t0;
  // dt
  output->dt = input->dt;
  // horizon
  output->horizon = input->horizon;
  // xdim
  output->xdim = input->xdim;
  // udim
  output->udim = input->udim;
  // max_it
  output->max_it = input->max_it;
  // params_json
  if (!rosidl_runtime_c__String__copy(
      &(input->params_json), &(output->params_json)))
  {
    return false;
  }
  return true;
}

ff_srvs__srv__PathPlan_Request *
ff_srvs__srv__PathPlan_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Request * msg = (ff_srvs__srv__PathPlan_Request *)allocator.allocate(sizeof(ff_srvs__srv__PathPlan_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_srvs__srv__PathPlan_Request));
  bool success = ff_srvs__srv__PathPlan_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_srvs__srv__PathPlan_Request__destroy(ff_srvs__srv__PathPlan_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_srvs__srv__PathPlan_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_srvs__srv__PathPlan_Request__Sequence__init(ff_srvs__srv__PathPlan_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Request * data = NULL;

  if (size) {
    data = (ff_srvs__srv__PathPlan_Request *)allocator.zero_allocate(size, sizeof(ff_srvs__srv__PathPlan_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_srvs__srv__PathPlan_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_srvs__srv__PathPlan_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ff_srvs__srv__PathPlan_Request__Sequence__fini(ff_srvs__srv__PathPlan_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ff_srvs__srv__PathPlan_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ff_srvs__srv__PathPlan_Request__Sequence *
ff_srvs__srv__PathPlan_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Request__Sequence * array = (ff_srvs__srv__PathPlan_Request__Sequence *)allocator.allocate(sizeof(ff_srvs__srv__PathPlan_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_srvs__srv__PathPlan_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_srvs__srv__PathPlan_Request__Sequence__destroy(ff_srvs__srv__PathPlan_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_srvs__srv__PathPlan_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_srvs__srv__PathPlan_Request__Sequence__are_equal(const ff_srvs__srv__PathPlan_Request__Sequence * lhs, const ff_srvs__srv__PathPlan_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_srvs__srv__PathPlan_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_srvs__srv__PathPlan_Request__Sequence__copy(
  const ff_srvs__srv__PathPlan_Request__Sequence * input,
  ff_srvs__srv__PathPlan_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_srvs__srv__PathPlan_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_srvs__srv__PathPlan_Request * data =
      (ff_srvs__srv__PathPlan_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_srvs__srv__PathPlan_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_srvs__srv__PathPlan_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_srvs__srv__PathPlan_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `times`
// Member `states`
// Member `controls`
// Member `feedback`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ff_srvs__srv__PathPlan_Response__init(ff_srvs__srv__PathPlan_Response * msg)
{
  if (!msg) {
    return false;
  }
  // times
  if (!rosidl_runtime_c__double__Sequence__init(&msg->times, 0)) {
    ff_srvs__srv__PathPlan_Response__fini(msg);
    return false;
  }
  // states
  if (!rosidl_runtime_c__double__Sequence__init(&msg->states, 0)) {
    ff_srvs__srv__PathPlan_Response__fini(msg);
    return false;
  }
  // controls
  if (!rosidl_runtime_c__double__Sequence__init(&msg->controls, 0)) {
    ff_srvs__srv__PathPlan_Response__fini(msg);
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__double__Sequence__init(&msg->feedback, 0)) {
    ff_srvs__srv__PathPlan_Response__fini(msg);
    return false;
  }
  return true;
}

void
ff_srvs__srv__PathPlan_Response__fini(ff_srvs__srv__PathPlan_Response * msg)
{
  if (!msg) {
    return;
  }
  // times
  rosidl_runtime_c__double__Sequence__fini(&msg->times);
  // states
  rosidl_runtime_c__double__Sequence__fini(&msg->states);
  // controls
  rosidl_runtime_c__double__Sequence__fini(&msg->controls);
  // feedback
  rosidl_runtime_c__double__Sequence__fini(&msg->feedback);
}

bool
ff_srvs__srv__PathPlan_Response__are_equal(const ff_srvs__srv__PathPlan_Response * lhs, const ff_srvs__srv__PathPlan_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // times
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->times), &(rhs->times)))
  {
    return false;
  }
  // states
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  // controls
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->controls), &(rhs->controls)))
  {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
ff_srvs__srv__PathPlan_Response__copy(
  const ff_srvs__srv__PathPlan_Response * input,
  ff_srvs__srv__PathPlan_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // times
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->times), &(output->times)))
  {
    return false;
  }
  // states
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  // controls
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->controls), &(output->controls)))
  {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

ff_srvs__srv__PathPlan_Response *
ff_srvs__srv__PathPlan_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Response * msg = (ff_srvs__srv__PathPlan_Response *)allocator.allocate(sizeof(ff_srvs__srv__PathPlan_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_srvs__srv__PathPlan_Response));
  bool success = ff_srvs__srv__PathPlan_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_srvs__srv__PathPlan_Response__destroy(ff_srvs__srv__PathPlan_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_srvs__srv__PathPlan_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_srvs__srv__PathPlan_Response__Sequence__init(ff_srvs__srv__PathPlan_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Response * data = NULL;

  if (size) {
    data = (ff_srvs__srv__PathPlan_Response *)allocator.zero_allocate(size, sizeof(ff_srvs__srv__PathPlan_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_srvs__srv__PathPlan_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_srvs__srv__PathPlan_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ff_srvs__srv__PathPlan_Response__Sequence__fini(ff_srvs__srv__PathPlan_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ff_srvs__srv__PathPlan_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ff_srvs__srv__PathPlan_Response__Sequence *
ff_srvs__srv__PathPlan_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_srvs__srv__PathPlan_Response__Sequence * array = (ff_srvs__srv__PathPlan_Response__Sequence *)allocator.allocate(sizeof(ff_srvs__srv__PathPlan_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_srvs__srv__PathPlan_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_srvs__srv__PathPlan_Response__Sequence__destroy(ff_srvs__srv__PathPlan_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_srvs__srv__PathPlan_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_srvs__srv__PathPlan_Response__Sequence__are_equal(const ff_srvs__srv__PathPlan_Response__Sequence * lhs, const ff_srvs__srv__PathPlan_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_srvs__srv__PathPlan_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_srvs__srv__PathPlan_Response__Sequence__copy(
  const ff_srvs__srv__PathPlan_Response__Sequence * input,
  ff_srvs__srv__PathPlan_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_srvs__srv__PathPlan_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_srvs__srv__PathPlan_Response * data =
      (ff_srvs__srv__PathPlan_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_srvs__srv__PathPlan_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_srvs__srv__PathPlan_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_srvs__srv__PathPlan_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
