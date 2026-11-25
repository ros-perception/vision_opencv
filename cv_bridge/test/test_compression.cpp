#include <cv_bridge/cv_bridge.hpp>
#include <stdio.h>
#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "opencv2/imgproc/imgproc.hpp"

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

  // Semi-planar YUV 4:2:0 images are converted to BGR before encoding, not compressed as the
  // raw stacked Y and UV planes.
  const std::pair<std::string, int> yuv420sp[] = {
    {"nv12", cv::COLOR_YUV2BGR_NV12}, {"nv21", cv::COLOR_YUV2BGR_NV21}};
  for (const auto & c : yuv420sp) {
    cv::Mat yuv(15, 10, CV_8UC1);  // 10 x 10 image
    rng.fill(yuv, cv::RNG::UNIFORM, 0, 256);
    cv::Mat expected;
    cv::cvtColor(yuv, expected, c.second);

    sensor_msgs::msg::CompressedImage::SharedPtr msg =
      cv_bridge::CvImage(header, c.first, yuv).toCompressedImageMsg(cv_bridge::PNG);
    const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(msg, "bgr8");

    EXPECT_EQ(out->image.rows, 10) << c.first;
    EXPECT_EQ(out->image.cols, 10) << c.first;
    EXPECT_EQ(out->image.channels(), 3) << c.first;
    EXPECT_EQ(cv::norm(out->image, expected), 0) << c.first;
  }
}
