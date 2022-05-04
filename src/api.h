#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CompoundJointIdentification_DLLIMPORT __declspec(dllimport)
#  define CompoundJointIdentification_DLLEXPORT __declspec(dllexport)
#  define CompoundJointIdentification_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CompoundJointIdentification_DLLIMPORT __attribute__((visibility("default")))
#    define CompoundJointIdentification_DLLEXPORT __attribute__((visibility("default")))
#    define CompoundJointIdentification_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CompoundJointIdentification_DLLIMPORT
#    define CompoundJointIdentification_DLLEXPORT
#    define CompoundJointIdentification_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CompoundJointIdentification_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CompoundJointIdentification_DLLAPI
#  define CompoundJointIdentification_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CompoundJointIdentification_EXPORTS
#    define CompoundJointIdentification_DLLAPI CompoundJointIdentification_DLLEXPORT
#  else
#    define CompoundJointIdentification_DLLAPI CompoundJointIdentification_DLLIMPORT
#  endif // CompoundJointIdentification_EXPORTS
#  define CompoundJointIdentification_LOCAL CompoundJointIdentification_DLLLOCAL
#endif // CompoundJointIdentification_STATIC