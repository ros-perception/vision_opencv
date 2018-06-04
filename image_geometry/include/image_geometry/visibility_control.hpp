// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef IMAGE_GEOMETRY__VISIBILITY_CONTROL_H_
#define IMAGE_GEOMETRY__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_GEOMETRY_EXPORT __attribute__ ((dllexport))
    #define IMAGE_GEOMETRY_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_GEOMETRY_EXPORT __declspec(dllexport)
    #define IMAGE_GEOMETRY_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_GEOMETRY_BUILDING_DLL
    #define IMAGE_GEOMETRY_PUBLIC IMAGE_GEOMETRY_EXPORT
  #else
    #define IMAGE_GEOMETRY_PUBLIC IMAGE_GEOMETRY_IMPORT
  #endif
  #define IMAGE_GEOMETRY_PUBLIC_TYPE IMAGE_GEOMETRY_PUBLIC
  #define IMAGE_GEOMETRY_LOCAL
#else
  #define IMAGE_GEOMETRY_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_GEOMETRY_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_GEOMETRY_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_GEOMETRY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_GEOMETRY_PUBLIC
    #define IMAGE_GEOMETRY_LOCAL
  #endif
  #define IMAGE_GEOMETRY_PUBLIC_TYPE
#endif

#if __cplusplus
}
#endif

#endif  // IMAGE_GEOMETRY__VISIBILITY_CONTROL_H_
