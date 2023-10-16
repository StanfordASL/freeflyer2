// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ff_msgs:msg/FreeFlyerStateStamped.idl
// generated code does not contain a copyright notice
#include "ff_msgs/msg/detail/free_flyer_state_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "ff_msgs/msg/detail/free_flyer_state__functions.h"

bool
ff_msgs__msg__FreeFlyerStateStamped__init(ff_msgs__msg__FreeFlyerStateStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ff_msgs__msg__FreeFlyerStateStamped__fini(msg);
    return false;
  }
  // state
  if (!ff_msgs__msg__FreeFlyerState__init(&msg->state)) {
    ff_msgs__msg__FreeFlyerStateStamped__fini(msg);
    return false;
  }
  return true;
}

void
ff_msgs__msg__FreeFlyerStateStamped__fini(ff_msgs__msg__FreeFlyerStateStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  ff_msgs__msg__FreeFlyerState__fini(&msg->state);
}

bool
ff_msgs__msg__FreeFlyerStateStamped__are_equal(const ff_msgs__msg__FreeFlyerStateStamped * lhs, const ff_msgs__msg__FreeFlyerStateStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // state
  if (!ff_msgs__msg__FreeFlyerState__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
ff_msgs__msg__FreeFlyerStateStamped__copy(
  const ff_msgs__msg__FreeFlyerStateStamped * input,
  ff_msgs__msg__FreeFlyerStateStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // state
  if (!ff_msgs__msg__FreeFlyerState__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

ff_msgs__msg__FreeFlyerStateStamped *
ff_msgs__msg__FreeFlyerStateStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerStateStamped * msg = (ff_msgs__msg__FreeFlyerStateStamped *)allocator.allocate(sizeof(ff_msgs__msg__FreeFlyerStateStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_msgs__msg__FreeFlyerStateStamped));
  bool success = ff_msgs__msg__FreeFlyerStateStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_msgs__msg__FreeFlyerStateStamped__destroy(ff_msgs__msg__FreeFlyerStateStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_msgs__msg__FreeFlyerStateStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_msgs__msg__FreeFlyerStateStamped__Sequence__init(ff_msgs__msg__FreeFlyerStateStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerStateStamped * data = NULL;

  if (size) {
    data = (ff_msgs__msg__FreeFlyerStateStamped *)allocator.zero_allocate(size, sizeof(ff_msgs__msg__FreeFlyerStateStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_msgs__msg__FreeFlyerStateStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_msgs__msg__FreeFlyerStateStamped__fini(&data[i - 1]);
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
ff_msgs__msg__FreeFlyerStateStamped__Sequence__fini(ff_msgs__msg__FreeFlyerStateStamped__Sequence * array)
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
      ff_msgs__msg__FreeFlyerStateStamped__fini(&array->data[i]);
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

ff_msgs__msg__FreeFlyerStateStamped__Sequence *
ff_msgs__msg__FreeFlyerStateStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__FreeFlyerStateStamped__Sequence * array = (ff_msgs__msg__FreeFlyerStateStamped__Sequence *)allocator.allocate(sizeof(ff_msgs__msg__FreeFlyerStateStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_msgs__msg__FreeFlyerStateStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_msgs__msg__FreeFlyerStateStamped__Sequence__destroy(ff_msgs__msg__FreeFlyerStateStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_msgs__msg__FreeFlyerStateStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_msgs__msg__FreeFlyerStateStamped__Sequence__are_equal(const ff_msgs__msg__FreeFlyerStateStamped__Sequence * lhs, const ff_msgs__msg__FreeFlyerStateStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_msgs__msg__FreeFlyerStateStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_msgs__msg__FreeFlyerStateStamped__Sequence__copy(
  const ff_msgs__msg__FreeFlyerStateStamped__Sequence * input,
  ff_msgs__msg__FreeFlyerStateStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_msgs__msg__FreeFlyerStateStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_msgs__msg__FreeFlyerStateStamped * data =
      (ff_msgs__msg__FreeFlyerStateStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_msgs__msg__FreeFlyerStateStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_msgs__msg__FreeFlyerStateStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_msgs__msg__FreeFlyerStateStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
