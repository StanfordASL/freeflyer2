// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ff_msgs:msg/FreeFlyerState.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_HPP_
#define FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "ff_msgs/msg/detail/pose2_d__struct.hpp"
// Member 'twist'
#include "ff_msgs/msg/detail/twist2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ff_msgs__msg__FreeFlyerState __attribute__((deprecated))
#else
# define DEPRECATED__ff_msgs__msg__FreeFlyerState __declspec(deprecated)
#endif

namespace ff_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FreeFlyerState_
{
  using Type = FreeFlyerState_<ContainerAllocator>;

  explicit FreeFlyerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    twist(_init)
  {
    (void)_init;
  }

  explicit FreeFlyerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    twist(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    ff_msgs::msg::Pose2D_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    ff_msgs::msg::Twist2D_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__pose(
    const ff_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const ff_msgs::msg::Twist2D_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ff_msgs::msg::FreeFlyerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const ff_msgs::msg::FreeFlyerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::FreeFlyerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::FreeFlyerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ff_msgs__msg__FreeFlyerState
    std::shared_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ff_msgs__msg__FreeFlyerState
    std::shared_ptr<ff_msgs::msg::FreeFlyerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FreeFlyerState_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const FreeFlyerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FreeFlyerState_

// alias to use template instance with default allocator
using FreeFlyerState =
  ff_msgs::msg::FreeFlyerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__FREE_FLYER_STATE__STRUCT_HPP_
