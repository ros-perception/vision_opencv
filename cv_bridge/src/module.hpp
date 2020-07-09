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

#ifndef MODULE_HPP_
#define MODULE_HPP_

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
#include <cv_bridge/cv_bridge.h>
#include <Python.h>

#include <boost/python.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

namespace bp = boost::python;

int convert_to_CvMat2(const PyObject * o, cv::Mat & m);

PyObject * pyopencv_from(const cv::Mat & m);

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wconversion-null"
# pragma clang diagnostic ignored "-Wreturn-type"
#elif __GNUC__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wconversion-null"
# pragma GCC diagnostic ignored "-Wreturn-type"
#endif

#if PYTHON3
static void * do_numpy_import()
{
  import_array();
  return NULL;
}
#else
static void do_numpy_import()
{
  import_array();
}
#endif

#ifdef __clang__
# pragma clang diagnostic pop
#elif __GNUC__
# pragma GCC diagnostic pop
#endif

#endif  // MODULE_HPP_
