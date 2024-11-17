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

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace enc = sensor_msgs::image_encodings;

bool isUnsigned(const std::string & encoding)
{
  const std::vector<std::string> unsigned_encodings = {
    enc::RGB8,
    enc::RGBA8,
    enc::RGB16,
    enc::RGBA16,
    enc::BGR8,
    enc::BGRA8,
    enc::BGR16,
    enc::BGRA16,
    enc::MONO8,
    enc::MONO16,
    enc::TYPE_8UC1,
    enc::TYPE_8UC2,
    enc::TYPE_8UC3,
    enc::TYPE_8UC4,
    enc::TYPE_16UC1,
    enc::TYPE_16UC2,
    enc::TYPE_16UC3,
    enc::TYPE_16UC4,
    // enc::BAYER_RGGB8,
    // enc::BAYER_BGGR8,
    // enc::BAYER_GBRG8,
    // enc::BAYER_GRBG8,
    // enc::BAYER_RGGB16,
    // enc::BAYER_BGGR16,
    // enc::BAYER_GBRG16,
    // enc::BAYER_GRBG16,
    // enc::YUV422
  };
  return std::find(unsigned_encodings.begin(), unsigned_encodings.end(), encoding) !=
         unsigned_encodings.end();
}

std::vector<std::string>
getEncodings()
{
  // TODO(N/A) for Groovy, the following types should be uncommented
  std::string encodings[] = {
    enc::RGB8,
    enc::RGBA8,
    enc::RGB16,
    enc::RGBA16,
    enc::BGR8,
    enc::BGRA8,
    enc::BGR16,
    enc::BGRA16,
    enc::MONO8,
    enc::MONO16,
    enc::TYPE_8UC1,
    // enc::TYPE_8UC2,
    enc::TYPE_8UC3,
    enc::TYPE_8UC4,
    enc::TYPE_8SC1,
    // enc::TYPE_8SC2,
    enc::TYPE_8SC3,
    enc::TYPE_8SC4,
    enc::TYPE_16UC1,
    // enc::TYPE_16UC2,
    enc::TYPE_16UC3,
    enc::TYPE_16UC4,
    enc::TYPE_16SC1,
    // enc::TYPE_16SC2,
    enc::TYPE_16SC3,
    enc::TYPE_16SC4,
    enc::TYPE_32SC1,
    // enc::TYPE_32SC2,
    enc::TYPE_32SC3,
    enc::TYPE_32SC4,
    enc::TYPE_32FC1,
    // enc::TYPE_32FC2,
    enc::TYPE_32FC3,
    enc::TYPE_32FC4,
    enc::TYPE_64FC1,
    // enc::TYPE_64FC2,
    enc::TYPE_64FC3,
    enc::TYPE_64FC4,
    // enc::BAYER_RGGB8,
    // enc::BAYER_BGGR8,
    // enc::BAYER_GBRG8,
    // enc::BAYER_GRBG8,
    // enc::BAYER_RGGB16,
    // enc::BAYER_BGGR16,
    // enc::BAYER_GBRG16,
    // enc::BAYER_GRBG16,
    enc::YUV422,
    enc::YUV422_YUY2
  };
  return std::vector<std::string>(encodings, encodings + 48 - 8 - 7);
}

TEST(OpencvTests, testCase_encode_decode)
{
  using enc::isColor;
  using enc::isMono;
  using enc::numChannels;
  using enc::bitDepth;

  std::vector<std::string> encodings = getEncodings();
  for (size_t i = 0; i < encodings.size(); ++i) {
    std::string src_encoding = encodings[i];
    bool is_src_color_format = isColor(src_encoding) || isMono(src_encoding) ||
      (src_encoding == enc::YUV422) ||
      (src_encoding == enc::YUV422_YUY2);
    cv::Mat image_original(cv::Size(400, 400), cv_bridge::getCvType(src_encoding));
    cv::RNG r(77);
    r.fill(image_original, cv::RNG::UNIFORM, 0, 127);

    sensor_msgs::msg::Image image_message;
    cv_bridge::CvImage image_bridge(std_msgs::msg::Header(), src_encoding, image_original);

    // Convert to a sensor_msgs::Image
    sensor_msgs::msg::Image::SharedPtr image_msg = image_bridge.toImageMsg();

    for (size_t j = 0; j < encodings.size(); ++j) {
      std::string dst_encoding = encodings[j];
      bool is_dst_color_format = isColor(dst_encoding) || isMono(dst_encoding) ||
        (dst_encoding == enc::YUV422) ||
        (dst_encoding == enc::YUV422_YUY2);
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
          cv_image = cv_bridge::toCvShare(image_msg, dst_encoding);
          // We cannot convert from non-color to color
          EXPECT_THROW((void)cvtColor(cv_image, src_encoding)->image, cv_bridge::Exception);
          continue;
        }
        // We do not support conversion to YUV422 for now, except from YUV422
        if (((dst_encoding == enc::YUV422) && (src_encoding != enc::YUV422)) ||
          ((dst_encoding == enc::YUV422_YUY2) && (src_encoding != enc::YUV422_YUY2)))
        {
          EXPECT_THROW(cv_bridge::toCvShare(image_msg, dst_encoding), cv_bridge::Exception);
          continue;
        }

        cv_image = cv_bridge::toCvShare(image_msg, dst_encoding);

        // We do not support conversion to YUV422 for now, except from YUV422
        if (((src_encoding == enc::YUV422) && (dst_encoding != enc::YUV422)) ||
          ((src_encoding == enc::YUV422_YUY2) && (dst_encoding != enc::YUV422_YUY2)))
        {
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
        EXPECT_LT(cv::norm(image_original, image_back, cv::NORM_INF),
          1) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else if ((bitDepth(src_encoding) == 16) && (bitDepth(dst_encoding) == 8)) {
        // In the case where the input has floats, we
        // will lose precision but no more than 1 * max(127)
        EXPECT_LT(cv::norm(image_original, image_back, cv::NORM_INF),
          128) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else {
        EXPECT_EQ(cv::norm(image_original, image_back, cv::NORM_INF),
          0) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      }
    }
  }
}
