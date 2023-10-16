// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ff_msgs:msg/Twist2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__TWIST2_D__FUNCTIONS_H_
#define FF_MSGS__MSG__DETAIL__TWIST2_D__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ff_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ff_msgs/msg/detail/twist2_d__struct.h"

/// Initialize msg/Twist2D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ff_msgs__msg__Twist2D
 * )) before or use
 * ff_msgs__msg__Twist2D__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__Twist2D__init(ff_msgs__msg__Twist2D * msg);

/// Finalize msg/Twist2D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__Twist2D__fini(ff_msgs__msg__Twist2D * msg);

/// Create msg/Twist2D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ff_msgs__msg__Twist2D__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
ff_msgs__msg__Twist2D *
ff_msgs__msg__Twist2D__create();

/// Destroy msg/Twist2D message.
/**
 * It calls
 * ff_msgs__msg__Twist2D__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__Twist2D__destroy(ff_msgs__msg__Twist2D * msg);

/// Check for msg/Twist2D message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__Twist2D__are_equal(const ff_msgs__msg__Twist2D * lhs, const ff_msgs__msg__Twist2D * rhs);

/// Copy a msg/Twist2D message.
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
ff_msgs__msg__Twist2D__copy(
  const ff_msgs__msg__Twist2D * input,
  ff_msgs__msg__Twist2D * output);

/// Initialize array of msg/Twist2D messages.
/**
 * It allocates the memory for the number of elements and calls
 * ff_msgs__msg__Twist2D__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__Twist2D__Sequence__init(ff_msgs__msg__Twist2D__Sequence * array, size_t size);

/// Finalize array of msg/Twist2D messages.
/**
 * It calls
 * ff_msgs__msg__Twist2D__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__Twist2D__Sequence__fini(ff_msgs__msg__Twist2D__Sequence * array);

/// Create array of msg/Twist2D messages.
/**
 * It allocates the memory for the array and calls
 * ff_msgs__msg__Twist2D__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
ff_msgs__msg__Twist2D__Sequence *
ff_msgs__msg__Twist2D__Sequence__create(size_t size);

/// Destroy array of msg/Twist2D messages.
/**
 * It calls
 * ff_msgs__msg__Twist2D__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
void
ff_msgs__msg__Twist2D__Sequence__destroy(ff_msgs__msg__Twist2D__Sequence * array);

/// Check for msg/Twist2D message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ff_msgs
bool
ff_msgs__msg__Twist2D__Sequence__are_equal(const ff_msgs__msg__Twist2D__Sequence * lhs, const ff_msgs__msg__Twist2D__Sequence * rhs);

/// Copy an array of msg/Twist2D messages.
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
ff_msgs__msg__Twist2D__Sequence__copy(
  const ff_msgs__msg__Twist2D__Sequence * input,
  ff_msgs__msg__Twist2D__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FF_MSGS__MSG__DETAIL__TWIST2_D__FUNCTIONS_H_
