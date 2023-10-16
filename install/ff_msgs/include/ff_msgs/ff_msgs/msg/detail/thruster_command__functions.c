// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ff_msgs:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice
#include "ff_msgs/msg/detail/thruster_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ff_msgs__msg__ThrusterCommand__init(ff_msgs__msg__ThrusterCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ff_msgs__msg__ThrusterCommand__fini(msg);
    return false;
  }
  // duty_cycle
  return true;
}

void
ff_msgs__msg__ThrusterCommand__fini(ff_msgs__msg__ThrusterCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // duty_cycle
}

bool
ff_msgs__msg__ThrusterCommand__are_equal(const ff_msgs__msg__ThrusterCommand * lhs, const ff_msgs__msg__ThrusterCommand * rhs)
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
  // duty_cycle
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->duty_cycle[i] != rhs->duty_cycle[i]) {
      return false;
    }
  }
  return true;
}

bool
ff_msgs__msg__ThrusterCommand__copy(
  const ff_msgs__msg__ThrusterCommand * input,
  ff_msgs__msg__ThrusterCommand * output)
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
  // duty_cycle
  for (size_t i = 0; i < 8; ++i) {
    output->duty_cycle[i] = input->duty_cycle[i];
  }
  return true;
}

ff_msgs__msg__ThrusterCommand *
ff_msgs__msg__ThrusterCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__ThrusterCommand * msg = (ff_msgs__msg__ThrusterCommand *)allocator.allocate(sizeof(ff_msgs__msg__ThrusterCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ff_msgs__msg__ThrusterCommand));
  bool success = ff_msgs__msg__ThrusterCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ff_msgs__msg__ThrusterCommand__destroy(ff_msgs__msg__ThrusterCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ff_msgs__msg__ThrusterCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ff_msgs__msg__ThrusterCommand__Sequence__init(ff_msgs__msg__ThrusterCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__ThrusterCommand * data = NULL;

  if (size) {
    data = (ff_msgs__msg__ThrusterCommand *)allocator.zero_allocate(size, sizeof(ff_msgs__msg__ThrusterCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ff_msgs__msg__ThrusterCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ff_msgs__msg__ThrusterCommand__fini(&data[i - 1]);
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
ff_msgs__msg__ThrusterCommand__Sequence__fini(ff_msgs__msg__ThrusterCommand__Sequence * array)
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
      ff_msgs__msg__ThrusterCommand__fini(&array->data[i]);
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

ff_msgs__msg__ThrusterCommand__Sequence *
ff_msgs__msg__ThrusterCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ff_msgs__msg__ThrusterCommand__Sequence * array = (ff_msgs__msg__ThrusterCommand__Sequence *)allocator.allocate(sizeof(ff_msgs__msg__ThrusterCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ff_msgs__msg__ThrusterCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ff_msgs__msg__ThrusterCommand__Sequence__destroy(ff_msgs__msg__ThrusterCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ff_msgs__msg__ThrusterCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ff_msgs__msg__ThrusterCommand__Sequence__are_equal(const ff_msgs__msg__ThrusterCommand__Sequence * lhs, const ff_msgs__msg__ThrusterCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ff_msgs__msg__ThrusterCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ff_msgs__msg__ThrusterCommand__Sequence__copy(
  const ff_msgs__msg__ThrusterCommand__Sequence * input,
  ff_msgs__msg__ThrusterCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ff_msgs__msg__ThrusterCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ff_msgs__msg__ThrusterCommand * data =
      (ff_msgs__msg__ThrusterCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ff_msgs__msg__ThrusterCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ff_msgs__msg__ThrusterCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ff_msgs__msg__ThrusterCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
