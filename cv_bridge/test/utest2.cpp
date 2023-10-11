/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <gtest/gtest.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core/core.hpp"

using namespace sensor_msgs::image_encodings;


struct EncodingProperty
{
  EncodingProperty(int num_channels, bool is_yuv, bool has_channel_info)
  : num_channels(num_channels), is_yuv(is_yuv), has_channel_info(has_channel_info)
  {
  }

  int num_channels;
  bool is_yuv;

  // Whether the channels can be assumed in the encoding. We cannot assume channels in opencv encodings.
  // Think about it this way: it does not make sense to convert 8UC3 to RGB as you don't know what
  // you store in your 3 channels (could be BGR, YUV whatever). Same here, maybe your 1 channel is
  // grayscale, maybe intensity, maybe it's not even color, it's a depth image.
  bool has_channel_info;
};

typedef std::map<std::string, EncodingProperty> EncodingProperties;
static EncodingProperties encodingProperties = {
  {RGB8, {3, false, true}},
  {RGBA8, {4, false, true}},
  {RGB16, {3, false, true}},
  {RGBA16, {4, false, true}},
  {BGR8, {3, false, true}},
  {BGRA8, {4, false, true}},
  {BGR16, {3, false, true}},
  {BGRA16, {4, false, true}},
  {MONO8, {1, false, true}},
  {MONO16, {1, false, true}},

  // OpenCV CvMat types
  {TYPE_8UC1, {1, false, false}},
  {TYPE_8UC2, {2, false, false}},
  {TYPE_8UC3, {3, false, false}},
  {TYPE_8UC4, {4, false, false}},
  {TYPE_8SC1, {1, false, false}},
  {TYPE_8SC2, {2, false, false}},
  {TYPE_8SC3, {3, false, false}},
  {TYPE_8SC4, {4, false, false}},
  {TYPE_16UC1, {1, false, false}},
  {TYPE_16UC2, {2, false, false}},
  {TYPE_16UC3, {3, false, false}},
  {TYPE_16UC4, {4, false, false}},
  {TYPE_16SC1, {1, false, false}},
  {TYPE_16SC2, {2, false, false}},
  {TYPE_16SC3, {3, false, false}},
  {TYPE_16SC4, {4, false, false}},
  {TYPE_32SC1, {1, false, false}},
  {TYPE_32SC2, {2, false, false}},
  {TYPE_32SC3, {3, false, false}},
  {TYPE_32SC4, {4, false, false}},
  {TYPE_32FC1, {1, false, false}},
  {TYPE_32FC2, {2, false, false}},
  {TYPE_32FC3, {3, false, false}},
  {TYPE_32FC4, {4, false, false}},
  {TYPE_64FC1, {1, false, false}},
  {TYPE_64FC2, {2, false, false}},
  {TYPE_64FC3, {3, false, false}},
  {TYPE_64FC4, {4, false, false}},

  // // Bayer encodings
  // {BAYER_RGGB8, },
  // {BAYER_BGGR8, },
  // {BAYER_GBRG8, },
  // {BAYER_GRBG8, },
  // {BAYER_RGGB16, },
  // {BAYER_BGGR16, },
  // {BAYER_GBRG16, },
  // {BAYER_GRBG16, },

  // YUV formats
  // YUV 4:2:2 encodings with an 8-bit depth
  // https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-packed-yuv.html#id1
  // fourcc: UYVY
  {UYVY, {2, true, true}},
  {YUV422, {2, true, true}},  // Deprecated
  // fourcc: YUYV
  {YUYV, {2, true, true}},
  {YUV422_YUY2, {2, true, true}},  // Deprecated

  // YUV 4:2:0 encodings with an 8-bit depth
  // NV21: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html#nv12-nv21-nv12m-and-nv21m
  {NV21, {2, true, true}},

  // YUV 4:4:4 encodings with 8-bit depth
  // NV24: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html#nv24-and-nv42
  {NV24, {2, true, true}},
};


TEST(OpencvTests, testCase_encode_decode)
{
  for (const auto & [src_encoding, src_encoding_property] : encodingProperties) {
    bool is_src_color_format = isColor(src_encoding) || isMono(src_encoding) ||
      src_encoding_property.is_yuv;
    cv::Mat image_original(cv::Size(400, 400), cv_bridge::getCvType(src_encoding));
    cv::RNG r(77);
    r.fill(image_original, cv::RNG::UNIFORM, 0, 127);

    sensor_msgs::msg::Image image_message;
    cv_bridge::CvImage image_bridge(std_msgs::msg::Header(), src_encoding, image_original);

    // Convert to a sensor_msgs::Image
    sensor_msgs::msg::Image::SharedPtr image_msg = image_bridge.toImageMsg();

    for (const auto & [dst_encoding, dst_encoding_property] : encodingProperties) {
      bool is_dst_color_format = isColor(dst_encoding) || isMono(dst_encoding) ||
        dst_encoding_property.is_yuv;
      bool is_num_channels_the_same = (numChannels(src_encoding) == numChannels(dst_encoding));

      cv_bridge::CvImageConstPtr cv_image;
      cv::Mat image_back;
      // If the first type does not contain any color information
      if (!is_src_color_format) {
        // Converting from a non color type to a color type does no make sense
        if (is_dst_color_format) {
          EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
          continue;
        }
        // We can only convert non-color types with the same number of channels
        if (!is_num_channels_the_same) {
          EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
          continue;
        }
        cv_image = cv_bridge::toCvShare(image_msg, dst_encoding);
      } else {
        // If we are converting to a non-color, you cannot convert to a different number of channels
        if (!is_dst_color_format) {
          if (!is_num_channels_the_same) {
            EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
            continue;
          }
          // We do not support conversion from Bayer for now
          if (isBayer(dst_encoding) && src_encoding != dst_encoding) {
            EXPECT_THROW(cv_bridge::toCvShare(image_msg, src_encoding), cv_bridge::Exception);
            continue;
          }
          cv_image = cv_bridge::toCvShare(image_msg, dst_encoding);
          // We cannot convert from non-color to color
          EXPECT_THROW((void)cvtColor(cv_image, src_encoding)->image, cv_bridge::Exception);
          continue;
        }
        // We do not support conversion to YUV for now
        if (dst_encoding_property.is_yuv && src_encoding != dst_encoding) {
          EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
          continue;
        }

        cv_image = cv_bridge::toCvShare(image_msg, dst_encoding);

        // We do not support conversion from YUV for now
        if (src_encoding_property.is_yuv && src_encoding != dst_encoding) {
          EXPECT_THROW((void)cvtColor(cv_image, src_encoding)->image, cv_bridge::Exception);
          continue;
        }
      }
      // And convert back to a cv::Mat
      image_back = cvtColor(cv_image, src_encoding)->image;

      // If the number of channels,s different some information
      // got lost at some point, so no possible test
      if (!is_num_channels_the_same) {
        continue;
      }
      if (bitDepth(src_encoding) >= 32) {
        // In the case where the input has floats, we will lose precision but no more than 1
        EXPECT_LT(
          cv::norm(image_original, image_back, cv::NORM_INF),
          1) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else if ((bitDepth(src_encoding) == 16) && (bitDepth(dst_encoding) == 8)) {
        // In the case where the input has floats, we
        // will lose precision but no more than 1 * max(127)
        EXPECT_LT(
          cv::norm(image_original, image_back, cv::NORM_INF),
          128) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else {
        EXPECT_EQ(
          cv::norm(image_original, image_back, cv::NORM_INF),
          0) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      }
    }
  }
}

static bool conversionPossible(const std::string & src_encoding, const std::string & dst_encoding)
{
  return true;
}

class TestCVConversion : public testing::TestWithParam<std::tuple<EncodingProperties::value_type,
    EncodingProperties::value_type>>
{
public:
  struct PrintToStringParamName
  {
    template<class ParamType>
    std::string operator()(const testing::TestParamInfo<ParamType> & info) const
    {
      const auto & [src_property, dst_property] = info.param;
      const auto & [src_encoding, _1] = src_property;
      const auto & [dst_encoding, _2] = dst_property;
      std::string name = src_encoding + "_to_" + dst_encoding;
      return name;
    }
  };
};

TEST_P(TestCVConversion, )
{
  const auto & [src_property, dst_property] = GetParam();
  const auto & [src_encoding, src_encoding_property] = src_property;
  const auto & [dst_encoding, dst_encoding_property] = dst_property;

  // Generate a random image
  cv::Mat image_original(cv::Size(400, 400), cv_bridge::getCvType(src_encoding));
  cv::RNG r(77);
  r.fill(image_original, cv::RNG::UNIFORM, 0, 127);

  sensor_msgs::msg::Image image_message;
  cv_bridge::CvImage image_bridge(std_msgs::msg::Header(), src_encoding, image_original);

  // Convert to a sensor_msgs::Image
  sensor_msgs::msg::Image::SharedPtr image_msg = image_bridge.toImageMsg();

  if (conversionPossible(src_encoding, dst_encoding)) {
    cv_bridge::CvImageConstPtr cv_image;
    EXPECT_NO_THROW(
      auto cv_image = cv_bridge::toCvShare(image_msg, dst_encoding)
    ) << "problem converting from " << src_encoding << " to " << dst_encoding;
  } else {
    EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
  }
}

static std::string generateName(const testing::TestParamInfo<TestCVConversion::ParamType>& info)
{
  const auto & [src_property, dst_property] = info.param;
  const auto & [src_encoding, _1] = src_property;
  const auto & [dst_encoding, _2] = dst_property;
  return src_encoding + "_to_" + dst_encoding;
}

INSTANTIATE_TEST_SUITE_P(
  ,
  TestCVConversion, testing::Combine(
    testing::ValuesIn(encodingProperties),
    testing::ValuesIn(encodingProperties)),
  generateName);
