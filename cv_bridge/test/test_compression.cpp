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

// Formats that store 8 bits get the 16 bit image scaled down, not saturated at 255.
TEST(CvBridgeTest, compression16bitTo8bitFormat)
{
  std_msgs::msg::Header header;
  cv::Mat_<uint16_t> in(8, 8);
  in.setTo(40000);

  for (const std::string src_encoding : {"mono16", "16UC1"}) {
    const sensor_msgs::msg::CompressedImage::SharedPtr msg =
      cv_bridge::CvImage(header, src_encoding, in).toCompressedImageMsg(cv_bridge::BMP);
    const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(msg, "");
    EXPECT_EQ(out->encoding, "mono8") << src_encoding;
    EXPECT_EQ(out->image.type(), CV_8UC1) << src_encoding;
    // 40000 * 255 / 65535
    EXPECT_EQ(out->image.at<uint8_t>(0, 0), 156) << src_encoding;
  }
}

// What is decoded is labelled with its real depth, also for color images.
TEST(CvBridgeTest, decompressionDepth)
{
  const auto compressed = [](const cv::Mat & image) {
      sensor_msgs::msg::CompressedImage msg;
      msg.format = "png";
      cv::imencode(".png", image, msg.data);
      return msg;
    };

  const cv::Mat bgr16(4, 4, CV_16UC3, cv::Scalar(1000, 20000, 65535));
  cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(compressed(bgr16), "");
  EXPECT_EQ(out->encoding, "bgr16");
  EXPECT_EQ(out->image.type(), CV_16UC3);
  EXPECT_EQ(cv::norm(out->image, bgr16, cv::NORM_INF), 0);

  out = cv_bridge::toCvCopy(compressed(bgr16), "bgr8");
  EXPECT_EQ(out->image.type(), CV_8UC3);
  EXPECT_EQ(out->image.at<cv::Vec3b>(0, 0), cv::Vec3b(4, 78, 255));

  const cv::Mat bgra16(4, 4, CV_16UC4, cv::Scalar(1000, 20000, 65535, 65535));
  out = cv_bridge::toCvCopy(compressed(bgra16), "");
  EXPECT_EQ(out->encoding, "bgra16");
  EXPECT_EQ(out->image.type(), CV_16UC4);

  // An 8 bit image requested as mono16 is converted, not just relabelled.
  const cv::Mat mono8(4, 4, CV_8UC1, cv::Scalar(200));
  out = cv_bridge::toCvCopy(compressed(mono8), "mono16");
  EXPECT_EQ(out->encoding, "mono16");
  EXPECT_EQ(out->image.type(), CV_16UC1);
  EXPECT_EQ(out->image.at<uint16_t>(0, 0), 200 * 65535 / 255);
}
