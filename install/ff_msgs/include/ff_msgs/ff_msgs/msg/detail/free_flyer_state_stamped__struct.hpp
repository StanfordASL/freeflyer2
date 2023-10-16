// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ff_msgs:msg/FreeFlyerStateStamped.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_HPP_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'state'
#include "ff_msgs/msg/detail/free_flyer_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ff_msgs__msg__FreeFlyerStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__ff_msgs__msg__FreeFlyerStateStamped __declspec(deprecated)
#endif

namespace ff_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FreeFlyerStateStamped_
{
  using Type = FreeFlyerStateStamped_<ContainerAllocator>;

  explicit FreeFlyerStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state(_init)
  {
    (void)_init;
  }

  explicit FreeFlyerStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    ff_msgs::msg::FreeFlyerState_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const ff_msgs::msg::FreeFlyerState_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ff_msgs__msg__FreeFlyerStateStamped
    std::shared_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ff_msgs__msg__FreeFlyerStateStamped
    std::shared_ptr<ff_msgs::msg::FreeFlyerStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FreeFlyerStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const FreeFlyerStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FreeFlyerStateStamped_

// alias to use template instance with default allocator
using FreeFlyerStateStamped =
  ff_msgs::msg::FreeFlyerStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE_STAMPED__STRUCT_HPP_
