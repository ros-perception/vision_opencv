// Copyright 2019 edmBernard
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

// This was sourced from https://github.com/edmBernard/pybind11_opencv_numpy/blob/7502afd33d70eb6342e6f30d1b745dc3e7cb6af9/ndarray_converter.h
// With some tweaks for ROS sytle.

# ifndef NDARRAY_CONVERTER_HPP__
# define NDARRAY_CONVERTER_HPP__

#include <Python.h>
#include <opencv2/core/core.hpp>


class NDArrayConverter
{
public:
  // must call this first, or the other routines don't work!
  static bool init_numpy();

  static bool toMat(PyObject * o, cv::Mat & m);
  static PyObject * toNDArray(const cv::Mat & mat);
};

//
// Define the type converter
//

#include <pybind11/pybind11.h>

namespace pybind11
{
namespace detail
{

template<>
struct type_caster<cv::Mat>
{
public:
  PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

  bool load(handle src, bool)
  {
    return NDArrayConverter::toMat(src.ptr(), value);
  }

  static handle cast(const cv::Mat & m, return_value_policy, handle /*defval*/)
  {
    return handle(NDArrayConverter::toNDArray(m));
  }
};

}  // namespace detail
}  // namespace pybind11

# endif
