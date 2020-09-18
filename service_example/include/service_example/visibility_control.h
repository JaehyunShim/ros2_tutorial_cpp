// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Authors: Ryan Shim */

#ifndef SERVICE_EXAMPLE__VISIBILITY_CONTROL_H_
#define SERVICE_EXAMPLE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SERVICE_EXAMPLE_EXPORT __attribute__ ((dllexport))
    #define SERVICE_EXAMPLE_IMPORT __attribute__ ((dllimport))
  #else
    #define SERVICE_EXAMPLE_EXPORT __declspec(dllexport)
    #define SERVICE_EXAMPLE_IMPORT __declspec(dllimport)
  #endif
  #ifdef SERVICE_EXAMPLE_BUILDING_DLL
    #define SERVICE_EXAMPLE_PUBLIC SERVICE_EXAMPLE_EXPORT
  #else
    #define SERVICE_EXAMPLE_PUBLIC SERVICE_EXAMPLE_IMPORT
  #endif
  #define SERVICE_EXAMPLE_PUBLIC_TYPE SERVICE_EXAMPLE_PUBLIC
  #define SERVICE_EXAMPLE_LOCAL
#else
  #define SERVICE_EXAMPLE_EXPORT __attribute__ ((visibility("default")))
  #define SERVICE_EXAMPLE_IMPORT
  #if __GNUC__ >= 4
    #define SERVICE_EXAMPLE_PUBLIC __attribute__ ((visibility("default")))
    #define SERVICE_EXAMPLE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SERVICE_EXAMPLE_PUBLIC
    #define SERVICE_EXAMPLE_LOCAL
  #endif
  #define SERVICE_EXAMPLE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_EXAMPLE__VISIBILITY_CONTROL_H_
