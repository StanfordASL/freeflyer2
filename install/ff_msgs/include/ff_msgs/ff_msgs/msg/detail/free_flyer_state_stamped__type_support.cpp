// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ff_msgs:msg/FreeFlyerStateStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ff_msgs/msg/detail/free_flyer_state_stamped__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ff_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FreeFlyerStateStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ff_msgs::msg::FreeFlyerStateStamped(_init);
}

void FreeFlyerStateStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ff_msgs::msg::FreeFlyerStateStamped *>(message_memory);
  typed_message->~FreeFlyerStateStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FreeFlyerStateStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_msgs::msg::FreeFlyerStateStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ff_msgs::msg::FreeFlyerState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ff_msgs::msg::FreeFlyerStateStamped, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FreeFlyerStateStamped_message_members = {
  "ff_msgs::msg",  // message namespace
  "FreeFlyerStateStamped",  // message name
  2,  // number of fields
  sizeof(ff_msgs::msg::FreeFlyerStateStamped),
  FreeFlyerStateStamped_message_member_array,  // message members
  FreeFlyerStateStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  FreeFlyerStateStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FreeFlyerStateStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FreeFlyerStateStamped_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ff_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ff_msgs::msg::FreeFlyerStateStamped>()
{
  return &::ff_msgs::msg::rosidl_typesupport_introspection_cpp::FreeFlyerStateStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ff_msgs, msg, FreeFlyerStateStamped)() {
  return &::ff_msgs::msg::rosidl_typesupport_introspection_cpp::FreeFlyerStateStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
