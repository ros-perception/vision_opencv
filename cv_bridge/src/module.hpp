// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef CV_BRIDGE_MODULE_HPP_
#define CV_BRIDGE_MODULE_HPP_

#include <iostream>

// Have to define macros to silence warnings about deprecated headers being used by
// boost/python.hpp in some versions of boost.
// See: https://github.com/ros-perception/vision_opencv/issues/449
#include <boost/version.hpp>
#if (BOOST_VERSION / 100 >= 1073 && BOOST_VERSION / 100 <= 1076)  // Boost 1.73 - 1.76
  #define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif
#if (BOOST_VERSION / 100 == 1074)  // Boost 1.74
  #define BOOST_ALLOW_DEPRECATED_HEADERS
#endif
#include <boost/python.hpp>

#include <cv_bridge/cv_bridge.h>
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

int convert_to_CvMat2(const PyObject * o, cv::Mat & m);

PyObject * pyopencv_from(const cv::Mat & m);

static void * do_numpy_import()
{
  import_array();
  return nullptr;
}

#endif  // CV_BRIDGE_MODULE_HPP_
