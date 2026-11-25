// Copyright (c) 2015, ROS Perception
#include <gtest/gtest.h>

#include <bit>
#include <memory>

#include <cv_bridge/cv_bridge.hpp>

TEST(CvBridgeTest, endianness)
{
  // Create an image of the type opposite to the platform
  sensor_msgs::msg::Image msg;
  msg.height = 1;
  msg.width = 1;
  msg.encoding = "32SC2";
  msg.step = 8;

  // Write 1 and 2 in order, but with an endianness opposite to the platform
  if (std::endian::native == std::endian::little) {
    msg.is_bigendian = true;
    msg.data = {0, 0, 0, 1, 0, 0, 0, 2};
  } else {
    msg.is_bigendian = false;
    msg.data = {1, 0, 0, 0, 2, 0, 0, 0};
  }

  // Make sure the values are still the same
  cv_bridge::CvImageConstPtr img =
    cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg));
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[0], 1);
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[1], 2);
  // Make sure we cannot share data
  EXPECT_NE(img->image.data, &msg.data[0]);
}
