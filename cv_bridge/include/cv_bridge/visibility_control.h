// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef CV_BRIDGE__VISIBILITY_CONTROL_H_
#define CV_BRIDGE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CV_BRIDGE_EXPORT __attribute__ ((dllexport))
    #define CV_BRIDGE_IMPORT __attribute__ ((dllimport))
  #else
    #define CV_BRIDGE_EXPORT __declspec(dllexport)
    #define CV_BRIDGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CV_BRIDGE_BUILDING_DLL
    #define CV_BRIDGE_PUBLIC CV_BRIDGE_EXPORT
  #else
    #define CV_BRIDGE_PUBLIC CV_BRIDGE_IMPORT
  #endif
  #define CV_BRIDGE_PUBLIC_TYPE CV_BRIDGE_PUBLIC
  #define CV_BRIDGE_LOCAL
#else
  #define CV_BRIDGE_EXPORT __attribute__ ((visibility("default")))
  #define CV_BRIDGE_IMPORT
  #if __GNUC__ >= 4
    #define CV_BRIDGE_PUBLIC __attribute__ ((visibility("default")))
    #define CV_BRIDGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CV_BRIDGE_PUBLIC
    #define CV_BRIDGE_LOCAL
  #endif
  #define CV_BRIDGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CV_BRIDGE__VISIBILITY_CONTROL_H_
