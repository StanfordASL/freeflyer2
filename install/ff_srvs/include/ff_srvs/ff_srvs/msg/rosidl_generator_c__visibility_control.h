// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef FF_SRVS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define FF_SRVS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_ff_srvs __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_ff_srvs __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_ff_srvs __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_ff_srvs __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_ff_srvs
    #define ROSIDL_GENERATOR_C_PUBLIC_ff_srvs ROSIDL_GENERATOR_C_EXPORT_ff_srvs
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_ff_srvs ROSIDL_GENERATOR_C_IMPORT_ff_srvs
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_ff_srvs __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_ff_srvs
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_ff_srvs __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_ff_srvs
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // FF_SRVS__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
