#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ff_control::ctrl_lib" for configuration ""
set_property(TARGET ff_control::ctrl_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ff_control::ctrl_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libctrl_lib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ff_control::ctrl_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_ff_control::ctrl_lib "${_IMPORT_PREFIX}/lib/libctrl_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
