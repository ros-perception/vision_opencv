// Copyright (c) 2018 Intel Corporation.
// All Rights Reserved.
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

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <gtest/gtest.h>

#include <string>

TEST(CvBridgeTest, compression)
{
  cv::RNG rng(0);
  std_msgs::msg::Header header;

  // Test 3 channel images.
  for (int i = 0; i < 2; ++i) {
    const std::string format = (i == 0) ? "bgr8" : "rgb8";
    cv::Mat_<cv::Vec3b> in(10, 10);
    rng.fill(in, cv::RNG::UNIFORM, 0, 256);

    sensor_msgs::msg::CompressedImage::SharedPtr msg =
      cv_bridge::CvImage(header, format, in).toCompressedImageMsg(cv_bridge::PNG);
    const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(msg, format);

    EXPECT_EQ(out->image.channels(), 3);
    EXPECT_EQ(cv::norm(out->image, in), 0);
  }

  // Test 4 channel images.
  for (int i = 0; i < 2; ++i) {
    const std::string format = (i == 0) ? "bgra8" : "rgba8";
    cv::Mat_<cv::Vec4b> in(10, 10);
    rng.fill(in, cv::RNG::UNIFORM, 0, 256);

    sensor_msgs::msg::CompressedImage::SharedPtr msg =
      cv_bridge::CvImage(header, format, in).toCompressedImageMsg(cv_bridge::PNG);
    const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(msg, format);
    EXPECT_EQ(out->image.channels(), 4);
    EXPECT_EQ(cv::norm(out->image, in), 0);
  }
}
