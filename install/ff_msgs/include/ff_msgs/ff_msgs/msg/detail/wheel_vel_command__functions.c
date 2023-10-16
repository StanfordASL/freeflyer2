// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ff_msgs:msg/WheelVelCommand.idl
// generated code does not contain a copyright notice
#include "ff_msgs/msg/detail/wheel_vel_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ff_msgs__msg__WheelVelCommand__init(ff_msgs__msg__WheelVelCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ff_msgs__msg__WheelVelCommand__fini(msg);
    return false;
  }
  // velocity
  return true;
}

void
ff_msgs__msg__WheelVelCommand__fini(ff_msgs__msg__WheelVelCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // velocity
}

bool
ff_msgs__msg__WheelVelCommand__are_equal(const ff_msgs__msg__WheelVelCommand * lhs, const ff_msgs__msg__WheelVelCommand * rhs)
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
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  return true;
}

bool
ff_msgs__msg__WheelVelCommand__copy(
  const ff_msgs__msg__WheelVelCommand * input,
  ff_msgs__msg__WheelVelCommand * output)
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
  // velocity
  output->velocity = input->velocity;
  return true;
}

ff_msgs__msg__WheelVelCommand *
ff_msgs__msg__WheelVelCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__WheelVelCommand * msg = (ff_msgs__msg__WheelVelCommand *)allocator.allocate(sizeof(ff_msgs__msg__WheelVelCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_msgs__msg__WheelVelCommand));
  bool success = ff_msgs__msg__WheelVelCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_msgs__msg__WheelVelCommand__destroy(ff_msgs__msg__WheelVelCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_msgs__msg__WheelVelCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_msgs__msg__WheelVelCommand__Sequence__init(ff_msgs__msg__WheelVelCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__WheelVelCommand * data = NULL;

  if (size) {
    data = (ff_msgs__msg__WheelVelCommand *)allocator.zero_allocate(size, sizeof(ff_msgs__msg__WheelVelCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_msgs__msg__WheelVelCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_msgs__msg__WheelVelCommand__fini(&data[i - 1]);
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
ff_msgs__msg__WheelVelCommand__Sequence__fini(ff_msgs__msg__WheelVelCommand__Sequence * array)
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
      ff_msgs__msg__WheelVelCommand__fini(&array->data[i]);
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

ff_msgs__msg__WheelVelCommand__Sequence *
ff_msgs__msg__WheelVelCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__WheelVelCommand__Sequence * array = (ff_msgs__msg__WheelVelCommand__Sequence *)allocator.allocate(sizeof(ff_msgs__msg__WheelVelCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_msgs__msg__WheelVelCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_msgs__msg__WheelVelCommand__Sequence__destroy(ff_msgs__msg__WheelVelCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_msgs__msg__WheelVelCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_msgs__msg__WheelVelCommand__Sequence__are_equal(const ff_msgs__msg__WheelVelCommand__Sequence * lhs, const ff_msgs__msg__WheelVelCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_msgs__msg__WheelVelCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_msgs__msg__WheelVelCommand__Sequence__copy(
  const ff_msgs__msg__WheelVelCommand__Sequence * input,
  ff_msgs__msg__WheelVelCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_msgs__msg__WheelVelCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_msgs__msg__WheelVelCommand * data =
      (ff_msgs__msg__WheelVelCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_msgs__msg__WheelVelCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_msgs__msg__WheelVelCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_msgs__msg__WheelVelCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
