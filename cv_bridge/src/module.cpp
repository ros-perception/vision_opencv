/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  Copyright (c) 2018 Intel Corporation.
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

#include "pybind11/pybind11.h"
#include "ndarray_converter.hpp"
#include "cv_bridge/cv_bridge.h"

#include <string>

namespace py = pybind11;

cv::Mat
cvtColor2Wrap(cv::Mat m, const std::string & encoding_in, const std::string & encoding_out)
{
  // Call cv_bridge for color conversion
  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
      std_msgs::msg::Header(), encoding_in, m));
  return cv_bridge::cvtColor(cv_image, encoding_out)->image;
}

cv::Mat
cvtColorForDisplayWrap(
  cv::Mat source,
  const std::string & encoding_in,
  const std::string & encoding_out,
  bool do_dynamic_scaling = false,
  double min_image_value = 0.0,
  double max_image_value = 0.0)
{
  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
      std_msgs::msg::Header(), encoding_in, source));

  cv_bridge::CvtColorForDisplayOptions options;
  options.do_dynamic_scaling = do_dynamic_scaling;
  options.min_image_value = min_image_value;
  options.max_image_value = max_image_value;
  cv::Mat mat_out = cv_bridge::cvtColorForDisplay(cv_image,
      encoding_out,
      options)->image;
  return mat_out;
}

int CV_MAT_CNWrap(int i)
{
  return CV_MAT_CN(i);
}

int CV_MAT_DEPTHWrap(int i)
{
  return CV_MAT_DEPTH(i);
}

PYBIND11_MODULE(cv_bridge_pybind11, m)
{
  NDArrayConverter::init_numpy();
  m.def("getCvType", &cv_bridge::getCvType,
    "Get the OpenCV type enum corresponding to the encoding.",
    py::arg("encoding"));
  m.def("cvtColor2", cvtColor2Wrap,
    "Convert an CvImage to another encoding using toCvCopy rules.",
    py::arg("m"), py::arg("encoding_in"), py::arg("encoding_out"));
  m.def("CV_MAT_CNWrap", CV_MAT_CNWrap,
    "Call CV_MAT_CN utility macro");
  m.def("CV_MAT_DEPTHWrap", CV_MAT_DEPTHWrap,
    "Call CV_MATH_DEPTH utility macro");
  m.def("cvtColorForDisplay", cvtColorForDisplayWrap,
    "Convert image to display with specified encodings.",
     py::arg("source"),
     py::arg("encoding_in"),
     py::arg("encoding_out"),
     py::arg("do_dynamic_scaling") = false,
     py::arg("min_image_value") = 0.0,
     py::arg("max_image_value") = 0.0
   );
}
