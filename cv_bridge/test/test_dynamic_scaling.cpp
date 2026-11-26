// Copyright (c) 2022, ROS Perception
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

TEST(TestDynamicScaling, ignoreInfAndNanValues)
{
  float inf = std::numeric_limits<float>::infinity();
  float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> data{50, 100, 150, -inf, inf, nan};
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  sensor_msgs::msg::Image msg;
  msg.height = 1;
  msg.width = data.size();
  msg.encoding = encoding;
  msg.step = data.size() * 4;
  for (auto d : data) {
    uint8_t * p = reinterpret_cast<uint8_t *>(&d);
    for (std::size_t i = 0; i != sizeof(float); ++i) {
      msg.data.push_back(p[i]);
    }
  }

  cv_bridge::CvtColorForDisplayOptions options;
  options.do_dynamic_scaling = true;

  cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg);
  auto converted = cv_bridge::cvtColorForDisplay(img, "", options);

  // Check that the scaling works for non-inf and non-nan values.
  std::vector<uint8_t> expected = {0, 0, 0, 128, 128, 128, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0,
    0};
  for (unsigned i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(converted->image.at<uint8_t>(i), expected.at(i));
  }
}

// Single channel floating point images are scaled for display without asking for it. #271
TEST(TestDynamicScaling, floatImagesScaledByDefault)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const cv::Mat_<float> depth = (cv::Mat_<float>(1, 4) << 0.5f, 1.0f, 1.5f, nan);
  const auto source = std::make_shared<cv_bridge::CvImage>(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_32FC1, depth);

  cv_bridge::CvImageConstPtr out;
  ASSERT_NO_THROW(out = cv_bridge::cvtColorForDisplay(source, "mono8"));
  EXPECT_EQ(out->encoding, "mono8");
  ASSERT_EQ(out->image.type(), CV_8UC1);
  EXPECT_EQ(out->image.at<uint8_t>(0), 0);
  EXPECT_EQ(out->image.at<uint8_t>(1), 128);
  EXPECT_EQ(out->image.at<uint8_t>(2), 255);
  EXPECT_EQ(out->image.at<uint8_t>(3), 0);

  // An empty output encoding gives a BGR image
  for (const std::string encoding : {"", "bgr8"}) {
    ASSERT_NO_THROW(out = cv_bridge::cvtColorForDisplay(source, encoding)) << encoding;
    EXPECT_EQ(out->encoding, "bgr8") << encoding;
    ASSERT_EQ(out->image.type(), CV_8UC3) << encoding;
    EXPECT_EQ(out->image.at<cv::Vec3b>(1), cv::Vec3b(128, 128, 128)) << encoding;
  }

  // 64 bit floats too
  cv::Mat depth64;
  depth.convertTo(depth64, CV_64F);
  ASSERT_NO_THROW(
    out = cv_bridge::cvtColorForDisplay(
      std::make_shared<cv_bridge::CvImage>(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_64FC1, depth64), "mono8"));
  EXPECT_EQ(out->image.at<uint8_t>(0), 0);
  EXPECT_EQ(out->image.at<uint8_t>(2), 255);

  // A range that is given is still used
  cv_bridge::CvtColorForDisplayOptions options;
  options.min_image_value = 0.0;
  options.max_image_value = 2.0;
  out = cv_bridge::cvtColorForDisplay(source, "mono8", options);
  EXPECT_EQ(out->image.at<uint8_t>(1), 128);
  EXPECT_EQ(out->image.at<uint8_t>(2), 191);

  // A constant image is shown as mid gray
  const auto flat = std::make_shared<cv_bridge::CvImage>(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_32FC1,
    cv::Mat(2, 2, CV_32FC1, cv::Scalar(3.0)));
  out = cv_bridge::cvtColorForDisplay(flat, "mono8");
  EXPECT_EQ(out->image.at<uint8_t>(0), 128);
}
