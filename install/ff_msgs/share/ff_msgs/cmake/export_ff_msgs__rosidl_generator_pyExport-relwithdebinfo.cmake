#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ff_msgs::ff_msgs__rosidl_generator_py" for configuration "RelWithDebInfo"
set_property(TARGET ff_msgs::ff_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ff_msgs::ff_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libff_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libff_msgs__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ff_msgs::ff_msgs__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_ff_msgs::ff_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libff_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
