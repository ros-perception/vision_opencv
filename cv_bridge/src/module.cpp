/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  Copyright (c) 2018 Intel Corporation.
*  Copyright (c) 2026 Open Source Robotics Foundation, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <climits>
#include <cstddef>
#include <cstdint>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "nanobind/nanobind.h"
#include "nanobind/ndarray.h"
#include "nanobind/stl/string.h"
#include "opencv2/core/core.hpp"

namespace nb = nanobind;

namespace
{

// The image as NumPy hands it over: any NumPy array that lives in main memory. nanobind makes a
// C contiguous copy first when the array is not (a slice with a step, a Fortran ordered array).
using InputArray = nb::ndarray<nb::numpy, nb::ro, nb::c_contig, nb::device::cpu>;
using OutputArray = nb::ndarray<nb::numpy>;

int depthFromDtype(const nb::dlpack::dtype & dtype)
{
  if (dtype == nb::dtype<uint8_t>()) {return CV_8U;}
  if (dtype == nb::dtype<int8_t>()) {return CV_8S;}
  if (dtype == nb::dtype<uint16_t>()) {return CV_16U;}
  if (dtype == nb::dtype<int16_t>()) {return CV_16S;}
  if (dtype == nb::dtype<int32_t>()) {return CV_32S;}
  if (dtype == nb::dtype<float>()) {return CV_32F;}
  if (dtype == nb::dtype<double>()) {return CV_64F;}
  throw nb::type_error(
          "Unsupported array dtype: expected uint8, int8, uint16, int16, int32, float32 or "
          "float64");
}

nb::dlpack::dtype dtypeFromDepth(int depth)
{
  switch (depth) {
    case CV_8U: return nb::dtype<uint8_t>();
    case CV_8S: return nb::dtype<int8_t>();
    case CV_16U: return nb::dtype<uint16_t>();
    case CV_16S: return nb::dtype<int16_t>();
    case CV_32S: return nb::dtype<int32_t>();
    case CV_32F: return nb::dtype<float>();
    case CV_64F: return nb::dtype<double>();
    default: throw cv_bridge::Exception("Unsupported cv::Mat depth");
  }
}

// Returns a cv::Mat that points to the memory of the array, without copying it.
// The array has to outlive the returned cv::Mat.
cv::Mat matFromArray(const InputArray & array)
{
  if (array.ndim() != 2 && array.ndim() != 3) {
    throw nb::type_error("Expected an array with 2 (rows, columns) or 3 (rows, columns, channels) "
            "dimensions");
  }
  const size_t rows = array.shape(0);
  const size_t cols = array.shape(1);
  const size_t channels = array.ndim() == 3 ? array.shape(2) : 1;
  if (rows > INT_MAX || cols > INT_MAX || channels < 1 || channels > CV_CN_MAX) {
    throw nb::value_error("The shape of the array cannot be represented as a cv::Mat");
  }
  const int type = CV_MAKETYPE(depthFromDtype(array.dtype()), static_cast<int>(channels));
  // cv::Mat has no read only flavour. The cv_bridge conversions do not write into their input.
  return cv::Mat(
    static_cast<int>(rows), static_cast<int>(cols), type, const_cast<void *>(array.data()));
}

// Returns a NumPy array that shares the memory of the image and keeps it alive.
OutputArray arrayFromMat(cv::Mat mat)
{
  if (mat.dims > 2) {
    throw cv_bridge::Exception("Only two dimensional cv::Mat can be converted to a NumPy array");
  }
  // NumPy needs one block that the returned array can own: copy when the image has padding
  // between rows or refers to memory that it does not own.
  if (!mat.isContinuous() || mat.u == nullptr) {
    mat = mat.clone();
  }
  const nb::dlpack::dtype dtype = dtypeFromDepth(mat.depth());

  cv::Mat * holder = new cv::Mat(mat);
  nb::capsule owner(holder, [](void * p) noexcept {delete static_cast<cv::Mat *>(p);});

  const size_t shape[3] = {
    static_cast<size_t>(mat.rows), static_cast<size_t>(mat.cols),
    static_cast<size_t>(mat.channels())};
  // As OpenCV does, images with one channel have two dimensions.
  const size_t ndim = mat.channels() == 1 ? 2 : 3;
  return OutputArray(holder->data, ndim, shape, owner, nullptr, dtype, nb::device::cpu::value);
}

OutputArray cvtColor2(
  const InputArray & source, const std::string & encoding_in, const std::string & encoding_out)
{
  cv::Mat converted;
  {
    // The conversion only uses the memory of the array, which the caller keeps alive.
    nb::gil_scoped_release release;
    cv_bridge::CvImagePtr image(new cv_bridge::CvImage(
        std_msgs::msg::Header(), encoding_in, matFromArray(source)));
    converted = cv_bridge::cvtColor(image, encoding_out)->image;
  }
  return arrayFromMat(converted);
}

OutputArray cvtColorForDisplay(
  const InputArray & source,
  const std::string & encoding_in,
  const std::string & encoding_out,
  nb::handle do_dynamic_scaling,
  double min_image_value,
  double max_image_value,
  int colormap)
{
  cv_bridge::CvtColorForDisplayOptions options;
  // Any object is taken by its truth value, callers used to pass 0 and 1.
  const int scaling = PyObject_IsTrue(do_dynamic_scaling.ptr());
  if (scaling < 0) {
    throw nb::python_error();
  }
  options.do_dynamic_scaling = scaling != 0;
  options.min_image_value = min_image_value;
  options.max_image_value = max_image_value;
  options.colormap = colormap;

  cv::Mat converted;
  {
    nb::gil_scoped_release release;
    cv_bridge::CvImagePtr image(new cv_bridge::CvImage(
        std_msgs::msg::Header(), encoding_in, matFromArray(source)));
    converted = cv_bridge::cvtColorForDisplay(image, encoding_out, options)->image;
  }
  return arrayFromMat(converted);
}

}  // namespace

NB_MODULE(_cv_bridge, m)
{
  m.doc() = "C++ part of the cv_bridge Python package";

  m.def(
    "getCvType", [](const std::string & encoding) {return cv_bridge::getCvType(encoding);},
    nb::arg("encoding"),
    "Get the OpenCV type enum corresponding to the encoding.");

  m.def(
    "cvtColor2", &cvtColor2,
    nb::arg("source"), nb::arg("encoding_in"), nb::arg("encoding_out"),
    "Convert an image from one encoding to another.");

  m.def("CV_MAT_CNWrap", [](int type) {return CV_MAT_CN(type);}, nb::arg("type"));
  m.def("CV_MAT_DEPTHWrap", [](int type) {return CV_MAT_DEPTH(type);}, nb::arg("type"));

  m.def(
    "cvtColorForDisplay", &cvtColorForDisplay,
    nb::arg("source"), nb::arg("encoding_in"), nb::arg("encoding_out"),
    nb::arg("do_dynamic_scaling") = false,
    nb::arg("min_image_value") = 0.0,
    nb::arg("max_image_value") = 0.0,
    nb::arg("colormap") = -1,
    "Convert image to display with specified encodings.\n\n"
    "Args:\n"
    "  - source (numpy.ndarray): input image\n"
    "  - encoding_in (str): input image encoding\n"
    "  - encoding_out (str): encoding to which the image conveted\n"
    "  - do_dynamic_scaling (bool): flag to do dynamic scaling with min/max value\n"
    "  - min_image_value (float): minimum pixel value for dynamic scaling\n"
    "  - max_image_value (float): maximum pixel value for dynamic scaling\n"
    "  - colormap (int): colormap to use when converting to color image\n");
}
