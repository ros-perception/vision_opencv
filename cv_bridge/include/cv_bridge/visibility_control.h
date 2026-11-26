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

// Kept for compatibility. New code should include cv_bridge/visibility_control.hpp.
#include "cv_bridge/visibility_control.hpp"

// No longer used by cv_bridge, still defined so that code using it keeps compiling.
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CV_BRIDGE_IMPORT __attribute__ ((dllimport))
  #else
    #define CV_BRIDGE_IMPORT __declspec(dllimport)
  #endif
#else
  #define CV_BRIDGE_IMPORT
#endif

#endif  // CV_BRIDGE__VISIBILITY_CONTROL_H_
