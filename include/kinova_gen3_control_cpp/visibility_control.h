#ifndef KINOVA_GEN3_CONTROL_CPP__VISIBILITY_CONTROL_H_
#define KINOVA_GEN3_CONTROL_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINOVA_GEN3_CONTROL_CPP_EXPORT __attribute__ ((dllexport))
    #define KINOVA_GEN3_CONTROL_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define KINOVA_GEN3_CONTROL_CPP_EXPORT __declspec(dllexport)
    #define KINOVA_GEN3_CONTROL_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINOVA_GEN3_CONTROL_CPP_BUILDING_DLL
    #define KINOVA_GEN3_CONTROL_CPP_PUBLIC KINOVA_GEN3_CONTROL_CPP_EXPORT
  #else
    #define KINOVA_GEN3_CONTROL_CPP_PUBLIC KINOVA_GEN3_CONTROL_CPP_IMPORT
  #endif
  #define KINOVA_GEN3_CONTROL_CPP_PUBLIC_TYPE KINOVA_GEN3_CONTROL_CPP_PUBLIC
  #define KINOVA_GEN3_CONTROL_CPP_LOCAL
#else
  #define KINOVA_GEN3_CONTROL_CPP_EXPORT __attribute__ ((visibility("default")))
  #define KINOVA_GEN3_CONTROL_CPP_IMPORT
  #if __GNUC__ >= 4
    #define KINOVA_GEN3_CONTROL_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define KINOVA_GEN3_CONTROL_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINOVA_GEN3_CONTROL_CPP_PUBLIC
    #define KINOVA_GEN3_CONTROL_CPP_LOCAL
  #endif
  #define KINOVA_GEN3_CONTROL_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // KINOVA_GEN3_CONTROL_CPP__VISIBILITY_CONTROL_H_