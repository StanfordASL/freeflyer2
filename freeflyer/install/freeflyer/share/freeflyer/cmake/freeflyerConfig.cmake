# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_freeflyer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED freeflyer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(freeflyer_FOUND FALSE)
  elseif(NOT freeflyer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(freeflyer_FOUND FALSE)
  endif()
  return()
endif()
set(_freeflyer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT freeflyer_FIND_QUIETLY)
  message(STATUS "Found freeflyer: 0.0.0 (${freeflyer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'freeflyer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${freeflyer_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(freeflyer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${freeflyer_DIR}/${_extra}")
endforeach()
