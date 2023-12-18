#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ff_srvs::ff_srvs__rosidl_typesupport_fastrtps_c" for configuration "RelWithDebInfo"
set_property(TARGET ff_srvs::ff_srvs__rosidl_typesupport_fastrtps_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ff_srvs::ff_srvs__rosidl_typesupport_fastrtps_c PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libff_srvs__rosidl_typesupport_fastrtps_c.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libff_srvs__rosidl_typesupport_fastrtps_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ff_srvs::ff_srvs__rosidl_typesupport_fastrtps_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_ff_srvs::ff_srvs__rosidl_typesupport_fastrtps_c "${_IMPORT_PREFIX}/lib/libff_srvs__rosidl_typesupport_fastrtps_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
