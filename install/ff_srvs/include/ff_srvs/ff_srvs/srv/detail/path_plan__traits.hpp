// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ff_srvs:srv/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef FF_SRVS__SRV__DETAIL__PATH_PLAN__TRAITS_HPP_
#define FF_SRVS__SRV__DETAIL__PATH_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ff_srvs/srv/detail/path_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ff_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PathPlan_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: dynamics
  {
    out << "dynamics: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamics, out);
    out << ", ";
  }

  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << ", ";
  }

  // member: x0
  {
    if (msg.x0.size() == 0) {
      out << "x0: []";
    } else {
      out << "x0: [";
      size_t pending_items = msg.x0.size();
      for (auto item : msg.x0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: t0
  {
    out << "t0: ";
    rosidl_generator_traits::value_to_yaml(msg.t0, out);
    out << ", ";
  }

  // member: dt
  {
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
    out << ", ";
  }

  // member: horizon
  {
    out << "horizon: ";
    rosidl_generator_traits::value_to_yaml(msg.horizon, out);
    out << ", ";
  }

  // member: xdim
  {
    out << "xdim: ";
    rosidl_generator_traits::value_to_yaml(msg.xdim, out);
    out << ", ";
  }

  // member: udim
  {
    out << "udim: ";
    rosidl_generator_traits::value_to_yaml(msg.udim, out);
    out << ", ";
  }

  // member: max_it
  {
    out << "max_it: ";
    rosidl_generator_traits::value_to_yaml(msg.max_it, out);
    out << ", ";
  }

  // member: params_json
  {
    out << "params_json: ";
    rosidl_generator_traits::value_to_yaml(msg.params_json, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dynamics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dynamics: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamics, out);
    out << "\n";
  }

  // member: cost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << "\n";
  }

  // member: x0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x0.size() == 0) {
      out << "x0: []\n";
    } else {
      out << "x0:\n";
      for (auto item : msg.x0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: t0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t0: ";
    rosidl_generator_traits::value_to_yaml(msg.t0, out);
    out << "\n";
  }

  // member: dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
    out << "\n";
  }

  // member: horizon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "horizon: ";
    rosidl_generator_traits::value_to_yaml(msg.horizon, out);
    out << "\n";
  }

  // member: xdim
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xdim: ";
    rosidl_generator_traits::value_to_yaml(msg.xdim, out);
    out << "\n";
  }

  // member: udim
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "udim: ";
    rosidl_generator_traits::value_to_yaml(msg.udim, out);
    out << "\n";
  }

  // member: max_it
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_it: ";
    rosidl_generator_traits::value_to_yaml(msg.max_it, out);
    out << "\n";
  }

  // member: params_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "params_json: ";
    rosidl_generator_traits::value_to_yaml(msg.params_json, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathPlan_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ff_srvs

namespace rosidl_generator_traits
{

[[deprecated("use ff_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ff_srvs::srv::PathPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ff_srvs::srv::PathPlan_Request & msg)
{
  return ff_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ff_srvs::srv::PathPlan_Request>()
{
  return "ff_srvs::srv::PathPlan_Request";
}

template<>
inline const char * name<ff_srvs::srv::PathPlan_Request>()
{
  return "ff_srvs/srv/PathPlan_Request";
}

template<>
struct has_fixed_size<ff_srvs::srv::PathPlan_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ff_srvs::srv::PathPlan_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ff_srvs::srv::PathPlan_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ff_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PathPlan_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: times
  {
    if (msg.times.size() == 0) {
      out << "times: []";
    } else {
      out << "times: [";
      size_t pending_items = msg.times.size();
      for (auto item : msg.times) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: controls
  {
    if (msg.controls.size() == 0) {
      out << "controls: []";
    } else {
      out << "controls: [";
      size_t pending_items = msg.controls.size();
      for (auto item : msg.controls) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feedback
  {
    if (msg.feedback.size() == 0) {
      out << "feedback: []";
    } else {
      out << "feedback: [";
      size_t pending_items = msg.feedback.size();
      for (auto item : msg.feedback) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: times
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.times.size() == 0) {
      out << "times: []\n";
    } else {
      out << "times:\n";
      for (auto item : msg.times) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: controls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.controls.size() == 0) {
      out << "controls: []\n";
    } else {
      out << "controls:\n";
      for (auto item : msg.controls) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feedback.size() == 0) {
      out << "feedback: []\n";
    } else {
      out << "feedback:\n";
      for (auto item : msg.feedback) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathPlan_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ff_srvs

namespace rosidl_generator_traits
{

[[deprecated("use ff_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ff_srvs::srv::PathPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ff_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ff_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ff_srvs::srv::PathPlan_Response & msg)
{
  return ff_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ff_srvs::srv::PathPlan_Response>()
{
  return "ff_srvs::srv::PathPlan_Response";
}

template<>
inline const char * name<ff_srvs::srv::PathPlan_Response>()
{
  return "ff_srvs/srv/PathPlan_Response";
}

template<>
struct has_fixed_size<ff_srvs::srv::PathPlan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ff_srvs::srv::PathPlan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ff_srvs::srv::PathPlan_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ff_srvs::srv::PathPlan>()
{
  return "ff_srvs::srv::PathPlan";
}

template<>
inline const char * name<ff_srvs::srv::PathPlan>()
{
  return "ff_srvs/srv/PathPlan";
}

template<>
struct has_fixed_size<ff_srvs::srv::PathPlan>
  : std::integral_constant<
    bool,
    has_fixed_size<ff_srvs::srv::PathPlan_Request>::value &&
    has_fixed_size<ff_srvs::srv::PathPlan_Response>::value
  >
{
};

template<>
struct has_bounded_size<ff_srvs::srv::PathPlan>
  : std::integral_constant<
    bool,
    has_bounded_size<ff_srvs::srv::PathPlan_Request>::value &&
    has_bounded_size<ff_srvs::srv::PathPlan_Response>::value
  >
{
};

template<>
struct is_service<ff_srvs::srv::PathPlan>
  : std::true_type
{
};

template<>
struct is_service_request<ff_srvs::srv::PathPlan_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ff_srvs::srv::PathPlan_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // FF_SRVS__SRV__DETAIL__PATH_PLAN__TRAITS_HPP_
