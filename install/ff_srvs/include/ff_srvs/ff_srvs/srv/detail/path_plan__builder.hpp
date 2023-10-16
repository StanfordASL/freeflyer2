// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef FF_SRVS__SRV__DETAIL__PATH_PLAN__BUILDER_HPP_
#define FF_SRVS__SRV__DETAIL__PATH_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ff_srvs/srv/detail/path_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ff_srvs
{

namespace srv
{

namespace builder
{

class Init_PathPlan_Request_params_json
{
public:
  explicit Init_PathPlan_Request_params_json(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  ::ff_srvs::srv::PathPlan_Request params_json(::ff_srvs::srv::PathPlan_Request::_params_json_type arg)
  {
    msg_.params_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_max_it
{
public:
  explicit Init_PathPlan_Request_max_it(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_params_json max_it(::ff_srvs::srv::PathPlan_Request::_max_it_type arg)
  {
    msg_.max_it = std::move(arg);
    return Init_PathPlan_Request_params_json(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_udim
{
public:
  explicit Init_PathPlan_Request_udim(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_max_it udim(::ff_srvs::srv::PathPlan_Request::_udim_type arg)
  {
    msg_.udim = std::move(arg);
    return Init_PathPlan_Request_max_it(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_xdim
{
public:
  explicit Init_PathPlan_Request_xdim(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_udim xdim(::ff_srvs::srv::PathPlan_Request::_xdim_type arg)
  {
    msg_.xdim = std::move(arg);
    return Init_PathPlan_Request_udim(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_horizon
{
public:
  explicit Init_PathPlan_Request_horizon(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_xdim horizon(::ff_srvs::srv::PathPlan_Request::_horizon_type arg)
  {
    msg_.horizon = std::move(arg);
    return Init_PathPlan_Request_xdim(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_dt
{
public:
  explicit Init_PathPlan_Request_dt(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_horizon dt(::ff_srvs::srv::PathPlan_Request::_dt_type arg)
  {
    msg_.dt = std::move(arg);
    return Init_PathPlan_Request_horizon(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_t0
{
public:
  explicit Init_PathPlan_Request_t0(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_dt t0(::ff_srvs::srv::PathPlan_Request::_t0_type arg)
  {
    msg_.t0 = std::move(arg);
    return Init_PathPlan_Request_dt(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_x0
{
public:
  explicit Init_PathPlan_Request_x0(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_t0 x0(::ff_srvs::srv::PathPlan_Request::_x0_type arg)
  {
    msg_.x0 = std::move(arg);
    return Init_PathPlan_Request_t0(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_cost
{
public:
  explicit Init_PathPlan_Request_cost(::ff_srvs::srv::PathPlan_Request & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Request_x0 cost(::ff_srvs::srv::PathPlan_Request::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return Init_PathPlan_Request_x0(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

class Init_PathPlan_Request_dynamics
{
public:
  Init_PathPlan_Request_dynamics()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_Request_cost dynamics(::ff_srvs::srv::PathPlan_Request::_dynamics_type arg)
  {
    msg_.dynamics = std::move(arg);
    return Init_PathPlan_Request_cost(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_srvs::srv::PathPlan_Request>()
{
  return ff_srvs::srv::builder::Init_PathPlan_Request_dynamics();
}

}  // namespace ff_srvs


namespace ff_srvs
{

namespace srv
{

namespace builder
{

class Init_PathPlan_Response_feedback
{
public:
  explicit Init_PathPlan_Response_feedback(::ff_srvs::srv::PathPlan_Response & msg)
  : msg_(msg)
  {}
  ::ff_srvs::srv::PathPlan_Response feedback(::ff_srvs::srv::PathPlan_Response::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Response msg_;
};

class Init_PathPlan_Response_controls
{
public:
  explicit Init_PathPlan_Response_controls(::ff_srvs::srv::PathPlan_Response & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Response_feedback controls(::ff_srvs::srv::PathPlan_Response::_controls_type arg)
  {
    msg_.controls = std::move(arg);
    return Init_PathPlan_Response_feedback(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Response msg_;
};

class Init_PathPlan_Response_states
{
public:
  explicit Init_PathPlan_Response_states(::ff_srvs::srv::PathPlan_Response & msg)
  : msg_(msg)
  {}
  Init_PathPlan_Response_controls states(::ff_srvs::srv::PathPlan_Response::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_PathPlan_Response_controls(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Response msg_;
};

class Init_PathPlan_Response_times
{
public:
  Init_PathPlan_Response_times()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_Response_states times(::ff_srvs::srv::PathPlan_Response::_times_type arg)
  {
    msg_.times = std::move(arg);
    return Init_PathPlan_Response_states(msg_);
  }

private:
  ::ff_srvs::srv::PathPlan_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ff_srvs::srv::PathPlan_Response>()
{
  return ff_srvs::srv::builder::Init_PathPlan_Response_times();
}

}  // namespace ff_srvs

#endif  // FF_SRVS__SRV__DETAIL__PATH_PLAN__BUILDER_HPP_
