// Copyright 2026 Open Source Robotics Foundation, Inc.
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
#include <stdexcept>

#include "cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp"

using Adapter = rclcpp::TypeAdapter<cv_bridge::ROSCvMatContainer, sensor_msgs::msg::Image>;

namespace
{
sensor_msgs::msg::Image makeImage(uint32_t width, uint32_t height)
{
  sensor_msgs::msg::Image image;
  image.encoding = "bgr8";
  image.width = width;
  image.height = height;
  image.step = width * 3;
  image.is_bigendian = false;
  image.data.resize(static_cast<size_t>(image.step) * height);
  for (size_t i = 0; i < image.data.size(); ++i) {
    image.data[i] = static_cast<uint8_t>(i);
  }
  return image;
}
}  // namespace

// A message received from the network must not produce a cv::Mat larger than its data
TEST(CvBridgeTypeAdapterTest, malformedImage)
{
  cv_bridge::ROSCvMatContainer container;

  // data is shorter than height * step
  sensor_msgs::msg::Image image = makeImage(100, 100);
  image.data.resize(16);
  EXPECT_THROW(Adapter::convert_to_custom(image, container), std::runtime_error);
  EXPECT_THROW(
    cv_bridge::ROSCvMatContainer(std::make_shared<sensor_msgs::msg::Image>(image)),
    std::runtime_error);

  // step is shorter than a row
  image = makeImage(100, 100);
  image.step = 100;
  image.data.resize(100 * 100);
  EXPECT_THROW(Adapter::convert_to_custom(image, container), std::runtime_error);

  // height * step wraps to data.size() in 32 bits
  image = makeImage(16384, 0);
  image.height = 65537;
  image.step = 65536;
  image.data.resize(65536);
  EXPECT_THROW(Adapter::convert_to_custom(image, container), std::runtime_error);

  EXPECT_THROW(
    cv_bridge::ROSCvMatContainer(std::shared_ptr<sensor_msgs::msg::Image>()),
    std::invalid_argument);
}

// A well formed message is still accepted, and its data is referenced by the cv::Mat
TEST(CvBridgeTypeAdapterTest, wellFormedImage)
{
  const sensor_msgs::msg::Image image = makeImage(4, 3);

  cv_bridge::ROSCvMatContainer container;
  ASSERT_NO_THROW(Adapter::convert_to_custom(image, container));
  ASSERT_EQ(container.cv_mat().rows, 3);
  ASSERT_EQ(container.cv_mat().cols, 4);
  EXPECT_EQ(container.cv_mat().type(), CV_8UC3);
  EXPECT_EQ(container.cv_mat().at<cv::Vec3b>(2, 3)[2], image.data.back());

  // A padded step is valid
  sensor_msgs::msg::Image padded = makeImage(4, 3);
  padded.step = 16;
  padded.data.resize(16 * 3);
  EXPECT_NO_THROW(Adapter::convert_to_custom(padded, container));
}
