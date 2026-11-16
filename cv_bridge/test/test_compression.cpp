#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
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

// 16 bit single channel images (e.g. depth) must stay 16 bit through PNG compression. #383
TEST(CvBridgeTest, compression16bit)
{
  cv::RNG rng(0);
  std_msgs::msg::Header header;

  cv::Mat_<uint16_t> in(10, 10);
  rng.fill(in, cv::RNG::UNIFORM, 0, 65536);
  in(0, 0) = 0;
  in(0, 1) = 2308;
  in(0, 2) = 65535;

  for (const std::string src_encoding : {"mono16", "16UC1"}) {
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    ASSERT_NO_THROW(
      msg = cv_bridge::CvImage(header, src_encoding, in).toCompressedImageMsg(cv_bridge::PNG))
      << src_encoding;

    // The payload itself is 16 bit
    const cv::Mat buffer(1, static_cast<int>(msg->data.size()), CV_8UC1, msg->data.data());
    const cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
    EXPECT_EQ(decoded.type(), CV_16UC1) << src_encoding;
    EXPECT_EQ(cv::norm(decoded, in), 0) << src_encoding;

    // Decoding through cv_bridge keeps the values, whatever 16 bit encoding is requested
    for (const std::string dst_encoding : {"", "mono16", "16UC1"}) {
      const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(msg, dst_encoding);
      ASSERT_TRUE(out) << src_encoding << " -> " << dst_encoding;
      EXPECT_EQ(out->image.type(), CV_16UC1) << src_encoding << " -> " << dst_encoding;
      EXPECT_EQ(cv::norm(out->image, in), 0) << src_encoding << " -> " << dst_encoding;
      EXPECT_EQ(out->encoding, dst_encoding.empty() ? std::string("mono16") : dst_encoding);
    }

    // Requesting mono8 scales the 16 bit range down to 8 bit
    const cv_bridge::CvImageConstPtr out8 = cv_bridge::toCvCopy(msg, "mono8");
    EXPECT_EQ(out8->image.type(), CV_8UC1);
    EXPECT_EQ(out8->image.at<uint8_t>(0, 2), 255);
  }
}
