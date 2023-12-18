#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ff_params::robot_params" for configuration "RelWithDebInfo"
set_property(TARGET ff_params::robot_params APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ff_params::robot_params PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/librobot_params.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ff_params::robot_params )
list(APPEND _IMPORT_CHECK_FILES_FOR_ff_params::robot_params "${_IMPORT_PREFIX}/lib/librobot_params.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
