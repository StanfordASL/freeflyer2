# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:ff_srvs__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:ff_srvs__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:ff_srvs__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:ff_srvs__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:ff_srvs__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:ff_srvs__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:ff_srvs__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:ff_srvs__rosidl_typesupport_cpp;__rosidl_generator_py:ff_srvs__rosidl_generator_py")

# populate ff_srvs_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "ff_srvs::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'ff_srvs' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND ff_srvs_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
