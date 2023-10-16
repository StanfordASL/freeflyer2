// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_HPP_
#define FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ff_srvs__srv__PathPlan_Request __attribute__((deprecated))
#else
# define DEPRECATED__ff_srvs__srv__PathPlan_Request __declspec(deprecated)
#endif

namespace ff_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PathPlan_Request_
{
  using Type = PathPlan_Request_<ContainerAllocator>;

  explicit PathPlan_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->cost = "default_cost";
      this->dt = 0.25;
      this->horizon = 20ll;
      this->xdim = -1ll;
      this->udim = -1ll;
      this->max_it = 20ll;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->dynamics = "";
      this->cost = "";
      this->t0 = 0.0;
      this->dt = 0.0;
      this->horizon = 0ll;
      this->xdim = 0ll;
      this->udim = 0ll;
      this->max_it = 0ll;
      this->params_json = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dynamics = "";
      this->t0 = 0.0;
      this->params_json = "";
    }
  }

  explicit PathPlan_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dynamics(_alloc),
    cost(_alloc),
    params_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->cost = "default_cost";
      this->dt = 0.25;
      this->horizon = 20ll;
      this->xdim = -1ll;
      this->udim = -1ll;
      this->max_it = 20ll;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->dynamics = "";
      this->cost = "";
      this->t0 = 0.0;
      this->dt = 0.0;
      this->horizon = 0ll;
      this->xdim = 0ll;
      this->udim = 0ll;
      this->max_it = 0ll;
      this->params_json = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dynamics = "";
      this->t0 = 0.0;
      this->params_json = "";
    }
  }

  // field types and members
  using _dynamics_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dynamics_type dynamics;
  using _cost_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cost_type cost;
  using _x0_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x0_type x0;
  using _t0_type =
    double;
  _t0_type t0;
  using _dt_type =
    double;
  _dt_type dt;
  using _horizon_type =
    int64_t;
  _horizon_type horizon;
  using _xdim_type =
    int64_t;
  _xdim_type xdim;
  using _udim_type =
    int64_t;
  _udim_type udim;
  using _max_it_type =
    int64_t;
  _max_it_type max_it;
  using _params_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _params_json_type params_json;

  // setters for named parameter idiom
  Type & set__dynamics(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dynamics = _arg;
    return *this;
  }
  Type & set__cost(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cost = _arg;
    return *this;
  }
  Type & set__x0(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x0 = _arg;
    return *this;
  }
  Type & set__t0(
    const double & _arg)
  {
    this->t0 = _arg;
    return *this;
  }
  Type & set__dt(
    const double & _arg)
  {
    this->dt = _arg;
    return *this;
  }
  Type & set__horizon(
    const int64_t & _arg)
  {
    this->horizon = _arg;
    return *this;
  }
  Type & set__xdim(
    const int64_t & _arg)
  {
    this->xdim = _arg;
    return *this;
  }
  Type & set__udim(
    const int64_t & _arg)
  {
    this->udim = _arg;
    return *this;
  }
  Type & set__max_it(
    const int64_t & _arg)
  {
    this->max_it = _arg;
    return *this;
  }
  Type & set__params_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->params_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ff_srvs::srv::PathPlan_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ff_srvs::srv::PathPlan_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ff_srvs::srv::PathPlan_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ff_srvs::srv::PathPlan_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ff_srvs__srv__PathPlan_Request
    std::shared_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ff_srvs__srv__PathPlan_Request
    std::shared_ptr<ff_srvs::srv::PathPlan_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathPlan_Request_ & other) const
  {
    if (this->dynamics != other.dynamics) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    if (this->x0 != other.x0) {
      return false;
    }
    if (this->t0 != other.t0) {
      return false;
    }
    if (this->dt != other.dt) {
      return false;
    }
    if (this->horizon != other.horizon) {
      return false;
    }
    if (this->xdim != other.xdim) {
      return false;
    }
    if (this->udim != other.udim) {
      return false;
    }
    if (this->max_it != other.max_it) {
      return false;
    }
    if (this->params_json != other.params_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathPlan_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathPlan_Request_

// alias to use template instance with default allocator
using PathPlan_Request =
  ff_srvs::srv::PathPlan_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ff_srvs


#ifndef _WIN32
# define DEPRECATED__ff_srvs__srv__PathPlan_Response __attribute__((deprecated))
#else
# define DEPRECATED__ff_srvs__srv__PathPlan_Response __declspec(deprecated)
#endif

namespace ff_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PathPlan_Response_
{
  using Type = PathPlan_Response_<ContainerAllocator>;

  explicit PathPlan_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PathPlan_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _times_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _times_type times;
  using _states_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _states_type states;
  using _controls_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _controls_type controls;
  using _feedback_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__times(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->times = _arg;
    return *this;
  }
  Type & set__states(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->states = _arg;
    return *this;
  }
  Type & set__controls(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->controls = _arg;
    return *this;
  }
  Type & set__feedback(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ff_srvs::srv::PathPlan_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ff_srvs::srv::PathPlan_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ff_srvs::srv::PathPlan_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ff_srvs::srv::PathPlan_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ff_srvs__srv__PathPlan_Response
    std::shared_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ff_srvs__srv__PathPlan_Response
    std::shared_ptr<ff_srvs::srv::PathPlan_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathPlan_Response_ & other) const
  {
    if (this->times != other.times) {
      return false;
    }
    if (this->states != other.states) {
      return false;
    }
    if (this->controls != other.controls) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathPlan_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathPlan_Response_

// alias to use template instance with default allocator
using PathPlan_Response =
  ff_srvs::srv::PathPlan_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ff_srvs

namespace ff_srvs
{

namespace srv
{

struct PathPlan
{
  using Request = ff_srvs::srv::PathPlan_Request;
  using Response = ff_srvs::srv::PathPlan_Response;
};

}  // namespace srv

}  // namespace ff_srvs

#endif  // FF_SRVS__SRV__DETAIL__PATH_PLAN__STRUCT_HPP_
