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

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <cv_bridge/cv_bridge.hpp>

namespace nb = nanobind;

// Helper function to convert nanobind ndarray to cv::Mat
cv::Mat convert_to_CvMat(nb::ndarray<nb::numpy> array)
{
    // Check if array is valid
    if (!array.is_valid())
    {
        throw std::runtime_error("Invalid numpy array");
    }

    size_t ndim = array.ndim();
    if (ndim < 2 || ndim > 3)
    {
        throw std::runtime_error("Input must be 2D or 3D array");
    }

    // Get shape information
    int rows = array.shape(0);
    int cols = array.shape(1);
    int channels = (ndim == 3) ? array.shape(2) : 1;

    // Determine OpenCV type from numpy dtype
    int cv_type;
    auto dtype = array.dtype();

    if (dtype == nb::dtype<uint8_t>())
    {
        cv_type = CV_8UC(channels);
    }
    else if (dtype == nb::dtype<int8_t>())
    {
        cv_type = CV_8SC(channels);
    }
    else if (dtype == nb::dtype<uint16_t>())
    {
        cv_type = CV_16UC(channels);
    }
    else if (dtype == nb::dtype<int16_t>())
    {
        cv_type = CV_16SC(channels);
    }
    else if (dtype == nb::dtype<int32_t>())
    {
        cv_type = CV_32SC(channels);
    }
    else if (dtype == nb::dtype<float>())
    {
        cv_type = CV_32FC(channels);
    }
    else if (dtype == nb::dtype<double>())
    {
        cv_type = CV_64FC(channels);
    }
    else
    {
        throw std::runtime_error("Unsupported numpy dtype");
    }

    // Create cv::Mat from numpy array data (clone to own the data)
    return cv::Mat(rows, cols, cv_type, const_cast<void *>(array.data())).clone();
}

// Helper function to convert cv::Mat to nanobind ndarray
nb::ndarray<nb::numpy> convert_from_CvMat(const cv::Mat &mat)
{
    // Build shape
    size_t ndim = (mat.channels() == 1) ? 2 : 3;
    size_t shape[3];

    shape[0] = static_cast<size_t>(mat.rows);
    shape[1] = static_cast<size_t>(mat.cols);
    if (ndim == 3)
    {
        shape[2] = static_cast<size_t>(mat.channels());
    }

    // Copy data to ensure it persists
    size_t size_bytes = static_cast<size_t>(mat.total()) * static_cast<size_t>(mat.elemSize());
    void *data = std::malloc(size_bytes);
    if (!data)
        throw std::bad_alloc();
    std::memcpy(data, mat.data, size_bytes);

    // Create capsule for memory management
    auto owner = nb::capsule(data, [](void *p) noexcept
                             { std::free(p); });

    // Create generic ndarray explicitly. Note: data pointer is first argument.
    switch (mat.depth())
    {
    case CV_8U:
        return nb::ndarray<nb::numpy>(
            data,                   // value (void*)
            ndim,                   // number of dimensions
            shape,                  // shape array (const size_t*)
            std::move(owner),       // owner/capsule (handle)
            nullptr,                // strides (optional)
            nb::dtype<uint8_t>(),   // dtype
            nb::device::cpu::value, // device
            0                       // readonly flag
        );
    case CV_8S:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<int8_t>(), nb::device::cpu::value, 0);
    case CV_16U:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<uint16_t>(), nb::device::cpu::value, 0);
    case CV_16S:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<int16_t>(), nb::device::cpu::value, 0);
    case CV_32S:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<int32_t>(), nb::device::cpu::value, 0);
    case CV_32F:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<float>(), nb::device::cpu::value, 0);
    case CV_64F:
        return nb::ndarray<nb::numpy>(data, ndim, shape, std::move(owner), nullptr, nb::dtype<double>(), nb::device::cpu::value, 0);
    default:
        std::free(data);
        throw std::runtime_error("Unsupported cv::Mat type");
    }
}

nb::ndarray<nb::numpy> cvtColor2Wrap(nb::ndarray<nb::numpy> obj_in, const std::string &encoding_in, const std::string &encoding_out)
{
    // Convert the Python input to an image
    cv::Mat mat_in = convert_to_CvMat(obj_in);

    // Call cv_bridge for color conversion
    cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
        std_msgs::msg::Header(), encoding_in, mat_in));

    cv::Mat mat = cv_bridge::cvtColor(cv_image, encoding_out)->image;

    return convert_from_CvMat(mat);
}

nb::ndarray<nb::numpy> cvtColorForDisplayWrap(
    nb::ndarray<nb::numpy> obj_in,
    const std::string &encoding_in,
    const std::string &encoding_out = "",
    bool do_dynamic_scaling = false,
    double min_image_value = 0.0,
    double max_image_value = 0.0,
    int colormap = -1)
{
    // Convert the numpy array input to cv::Mat
    cv::Mat mat_in = convert_to_CvMat(obj_in);

    // Create CvImage
    cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
        std_msgs::msg::Header(), encoding_in, mat_in));

    // Set up conversion options
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = do_dynamic_scaling;
    options.min_image_value = min_image_value;
    options.max_image_value = max_image_value;
    options.colormap = colormap;

    // Perform the conversion
    cv::Mat mat = cv_bridge::cvtColorForDisplay(
                      /*source=*/cv_image,
                      /*encoding_out=*/encoding_out,
                      /*options=*/options)
                      ->image;

    // Convert cv::Mat back to numpy array
    return convert_from_CvMat(mat);
}

int CV_MAT_CNWrap(int i)
{
    return CV_MAT_CN(i);
}

int CV_MAT_DEPTHWrap(int i)
{
    return CV_MAT_DEPTH(i);
}

NB_MODULE(cv_bridge_nanobind, m)
{
    m.doc() = "cv_bridge_nanobind: Lightweight nanobind bindings for ROS2 cv_bridge.\n\n"
              "Provides efficient conversion between NumPy arrays and OpenCV cv::Mat,\n"
              "plus wrappers for cv_bridge color conversions.\n\n"
              "Functions:\n"
              "  getCvType(encoding): Return OpenCV type for a ROS encoding.\n"
              "  cvtColor2(img, encoding_in, encoding_out): Convert image encoding.\n"
              "  cvtColorForDisplay(img, encoding_in, encoding_out='',\n"
              "                     do_dynamic_scaling=False,\n"
              "                     min_image_value=0, max_image_value=0,\n"
              "                     colormap=-1): Prepare image for visualization.\n"
              "  CV_MAT_CNWrap(type): Get number of channels from CV type.\n"
              "  CV_MAT_DEPTHWrap(type): Get depth (dtype) from CV type.\n";
    m.def("getCvType", &cv_bridge::getCvType, "Get the OpenCV type for a given encoding");
    m.def("cvtColor2", cvtColor2Wrap);
    m.def("CV_MAT_CNWrap", CV_MAT_CNWrap);
    m.def("CV_MAT_DEPTHWrap", CV_MAT_DEPTHWrap);
    m.def("cvtColorForDisplay",
          &cvtColorForDisplayWrap,
          nb::arg("source"),
          nb::arg("encoding_in"),
          nb::arg("encoding_out") = "",
          nb::arg("do_dynamic_scaling") = false,
          nb::arg("min_image_value") = 0.0,
          nb::arg("max_image_value") = 0.0,
          nb::arg("colormap") = -1,
          "Convert image to display with specified encodings.\n\n"
          "Args:\n"
          "  - source (numpy.ndarray): input image\n"
          "  - encoding_in (str): input image encoding\n"
          "  - encoding_out (str): encoding to which the image converted\n"
          "  - do_dynamic_scaling (bool): flag to do dynamic scaling with min/max value\n"
          "  - min_image_value (float): minimum pixel value for dynamic scaling\n"
          "  - max_image_value (float): maximum pixel value for dynamic scaling\n"
          "  - colormap (int): colormap to use when converting to color image\n");
}