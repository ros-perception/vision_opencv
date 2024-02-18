// Copyright 2023 Open Source Robotics Foundation, Inc.
// Copyright 2023 Homalozoa, Direct Drive Technology, Inc.
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

#include <gtest/gtest.h>

#include <memory>

#include "boost/endian/conversion.hpp"
#include "cv_bridge/cv_bridge.hpp"

TEST(CvBridgeTest, endianness)
{
  // Create an image of the type opposite to the platform
  sensor_msgs::msg::Image msg;
  msg.height = 1;
  msg.width = 1;
  msg.encoding = "32SC2";
  msg.step = 8;

  msg.data.resize(msg.step);
  int32_t * data = reinterpret_cast<int32_t *>(&msg.data[0]);

  // Write 1 and 2 in order, but with an endianness opposite to the platform
  if (boost::endian::order::native == boost::endian::order::little) {
    msg.is_bigendian = true;
    *(data++) = boost::endian::native_to_big(static_cast<int32_t>(1));
    *data = boost::endian::native_to_big(static_cast<int32_t>(2));
  } else {
    msg.is_bigendian = false;
    *(data++) = boost::endian::native_to_little(static_cast<int32_t>(1));
    *data = boost::endian::native_to_little(static_cast<int32_t>(2));
  }

  // Make sure the values are still the same
  cv_bridge::CvImageConstPtr img =
    cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg));
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[0], 1);
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[1], 2);
  // Make sure we cannot share data
  EXPECT_NE(img->image.data, &msg.data[0]);
}
