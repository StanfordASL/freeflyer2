// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ff_msgs:msg/WheelVelCommand.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__FUNCTIONS_H_
#define FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ff_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ff_msgs/msg/detail/wheel_vel_command__struct.h"

/// Initialize msg/WheelVelCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ff_msgs__msg__WheelVelCommand
 * )) before or use
 * ff_msgs__msg__WheelVelCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__init(ff_msgs__msg__WheelVelCommand * msg);

/// Finalize msg/WheelVelCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__WheelVelCommand__fini(ff_msgs__msg__WheelVelCommand * msg);

/// Create msg/WheelVelCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ff_msgs__msg__WheelVelCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
ff_msgs__msg__WheelVelCommand *
ff_msgs__msg__WheelVelCommand__create();

/// Destroy msg/WheelVelCommand message.
/**
 * It calls
 * ff_msgs__msg__WheelVelCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__WheelVelCommand__destroy(ff_msgs__msg__WheelVelCommand * msg);

/// Check for msg/WheelVelCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__are_equal(const ff_msgs__msg__WheelVelCommand * lhs, const ff_msgs__msg__WheelVelCommand * rhs);

/// Copy a msg/WheelVelCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__copy(
  const ff_msgs__msg__WheelVelCommand * input,
  ff_msgs__msg__WheelVelCommand * output);

/// Initialize array of msg/WheelVelCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * ff_msgs__msg__WheelVelCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__Sequence__init(ff_msgs__msg__WheelVelCommand__Sequence * array, size_t size);

/// Finalize array of msg/WheelVelCommand messages.
/**
 * It calls
 * ff_msgs__msg__WheelVelCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__WheelVelCommand__Sequence__fini(ff_msgs__msg__WheelVelCommand__Sequence * array);

/// Create array of msg/WheelVelCommand messages.
/**
 * It allocates the memory for the array and calls
 * ff_msgs__msg__WheelVelCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
ff_msgs__msg__WheelVelCommand__Sequence *
ff_msgs__msg__WheelVelCommand__Sequence__create(size_t size);

/// Destroy array of msg/WheelVelCommand messages.
/**
 * It calls
 * ff_msgs__msg__WheelVelCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__WheelVelCommand__Sequence__destroy(ff_msgs__msg__WheelVelCommand__Sequence * array);

/// Check for msg/WheelVelCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__Sequence__are_equal(const ff_msgs__msg__WheelVelCommand__Sequence * lhs, const ff_msgs__msg__WheelVelCommand__Sequence * rhs);

/// Copy an array of msg/WheelVelCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__WheelVelCommand__Sequence__copy(
  const ff_msgs__msg__WheelVelCommand__Sequence * input,
  ff_msgs__msg__WheelVelCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__WHEEL_VEL_COMMAND__FUNCTIONS_H_
