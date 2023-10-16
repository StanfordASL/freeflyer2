// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ff_msgs:msg/Wrench2D.idl
// generated code does not contain a copyright notice

#ifndef FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_HPP_
#define FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ff_msgs__msg__Wrench2D __attribute__((deprecated))
#else
# define DEPRECATED__ff_msgs__msg__Wrench2D __declspec(deprecated)
#endif

namespace ff_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Wrench2D_
{
  using Type = Wrench2D_<ContainerAllocator>;

  explicit Wrench2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx = 0.0;
      this->fy = 0.0;
      this->tz = 0.0;
    }
  }

  explicit Wrench2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx = 0.0;
      this->fy = 0.0;
      this->tz = 0.0;
    }
  }

  // field types and members
  using _fx_type =
    double;
  _fx_type fx;
  using _fy_type =
    double;
  _fy_type fy;
  using _tz_type =
    double;
  _tz_type tz;

  // setters for named parameter idiom
  Type & set__fx(
    const double & _arg)
  {
    this->fx = _arg;
    return *this;
  }
  Type & set__fy(
    const double & _arg)
  {
    this->fy = _arg;
    return *this;
  }
  Type & set__tz(
    const double & _arg)
  {
    this->tz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ff_msgs::msg::Wrench2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const ff_msgs::msg::Wrench2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::Wrench2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ff_msgs::msg::Wrench2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ff_msgs__msg__Wrench2D
    std::shared_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ff_msgs__msg__Wrench2D
    std::shared_ptr<ff_msgs::msg::Wrench2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Wrench2D_ & other) const
  {
    if (this->fx != other.fx) {
      return false;
    }
    if (this->fy != other.fy) {
      return false;
    }
    if (this->tz != other.tz) {
      return false;
    }
    return true;
  }
  bool operator!=(const Wrench2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Wrench2D_

// alias to use template instance with default allocator
using Wrench2D =
  ff_msgs::msg::Wrench2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ff_msgs

#endif  // FF_MSGS__MSG__DETAIL__WRENCH2_D__STRUCT_HPP_
