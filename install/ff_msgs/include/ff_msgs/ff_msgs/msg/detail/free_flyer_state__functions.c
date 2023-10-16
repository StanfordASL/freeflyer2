// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ff_msgs:msg/FreeFlyerState.idl
// generated code does not contain a copyright notice
#include "ff_msgs/msg/detail/free_flyer_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "ff_msgs/msg/detail/pose2_d__functions.h"
// Member `twist`
#include "ff_msgs/msg/detail/twist2_d__functions.h"

bool
ff_msgs__msg__FreeFlyerState__init(ff_msgs__msg__FreeFlyerState * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!ff_msgs__msg__Pose2D__init(&msg->pose)) {
    ff_msgs__msg__FreeFlyerState__fini(msg);
    return false;
  }
  // twist
  if (!ff_msgs__msg__Twist2D__init(&msg->twist)) {
    ff_msgs__msg__FreeFlyerState__fini(msg);
    return false;
  }
  return true;
}

void
ff_msgs__msg__FreeFlyerState__fini(ff_msgs__msg__FreeFlyerState * msg)
{
  if (!msg) {
    return;
  }
  // pose
  ff_msgs__msg__Pose2D__fini(&msg->pose);
  // twist
  ff_msgs__msg__Twist2D__fini(&msg->twist);
}

bool
ff_msgs__msg__FreeFlyerState__are_equal(const ff_msgs__msg__FreeFlyerState * lhs, const ff_msgs__msg__FreeFlyerState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!ff_msgs__msg__Pose2D__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!ff_msgs__msg__Twist2D__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  return true;
}

bool
ff_msgs__msg__FreeFlyerState__copy(
  const ff_msgs__msg__FreeFlyerState * input,
  ff_msgs__msg__FreeFlyerState * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!ff_msgs__msg__Pose2D__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!ff_msgs__msg__Twist2D__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  return true;
}

ff_msgs__msg__FreeFlyerState *
ff_msgs__msg__FreeFlyerState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerState * msg = (ff_msgs__msg__FreeFlyerState *)allocator.allocate(sizeof(ff_msgs__msg__FreeFlyerState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_msgs__msg__FreeFlyerState));
  bool success = ff_msgs__msg__FreeFlyerState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_msgs__msg__FreeFlyerState__destroy(ff_msgs__msg__FreeFlyerState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_msgs__msg__FreeFlyerState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_msgs__msg__FreeFlyerState__Sequence__init(ff_msgs__msg__FreeFlyerState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerState * data = NULL;

  if (size) {
    data = (ff_msgs__msg__FreeFlyerState *)allocator.zero_allocate(size, sizeof(ff_msgs__msg__FreeFlyerState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_msgs__msg__FreeFlyerState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_msgs__msg__FreeFlyerState__fini(&data[i - 1]);
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
ff_msgs__msg__FreeFlyerState__Sequence__fini(ff_msgs__msg__FreeFlyerState__Sequence * array)
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
      ff_msgs__msg__FreeFlyerState__fini(&array->data[i]);
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

ff_msgs__msg__FreeFlyerState__Sequence *
ff_msgs__msg__FreeFlyerState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerState__Sequence * array = (ff_msgs__msg__FreeFlyerState__Sequence *)allocator.allocate(sizeof(ff_msgs__msg__FreeFlyerState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_msgs__msg__FreeFlyerState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_msgs__msg__FreeFlyerState__Sequence__destroy(ff_msgs__msg__FreeFlyerState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_msgs__msg__FreeFlyerState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_msgs__msg__FreeFlyerState__Sequence__are_equal(const ff_msgs__msg__FreeFlyerState__Sequence * lhs, const ff_msgs__msg__FreeFlyerState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_msgs__msg__FreeFlyerState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_msgs__msg__FreeFlyerState__Sequence__copy(
  const ff_msgs__msg__FreeFlyerState__Sequence * input,
  ff_msgs__msg__FreeFlyerState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_msgs__msg__FreeFlyerState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_msgs__msg__FreeFlyerState * data =
      (ff_msgs__msg__FreeFlyerState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_msgs__msg__FreeFlyerState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_msgs__msg__FreeFlyerState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_msgs__msg__FreeFlyerState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
