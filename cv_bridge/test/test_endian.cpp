#include <cv_bridge/cv_bridge.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <rcpputils/endian.hpp>

TEST(CvBridgeTest, endianness)
{
  // Create an image of the type opposite to the platform
  sensor_msgs::msg::Image msg;
  msg.height = 1;
  msg.width = 1;
  msg.encoding = "32SC2";
  msg.step = 8;

  msg.data.resize(msg.step);
  uint8_t * raw = msg.data.data();

  // Write 1 and 2 in order, but with an endianness opposite to the platform
  if (rcpputils::endian::native == rcpputils::endian::little) {
    msg.is_bigendian = true;
    raw[0] = 0x00;
    raw[1] = 0x00;
    raw[2] = 0x00;
    raw[3] = 0x01;

    raw[4] = 0x00;
    raw[5] = 0x00;
    raw[6] = 0x00;
    raw[7] = 0x02;
  } else {
    msg.is_bigendian = false;
    raw[0] = 0x01;
    raw[1] = 0x00;
    raw[2] = 0x00;
    raw[3] = 0x00;

    raw[4] = 0x02;
    raw[5] = 0x00;
    raw[6] = 0x00;
    raw[7] = 0x00;
  }

  // Make sure the values are still the same
  cv_bridge::CvImageConstPtr img =
    cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg));
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[0], 1);
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[1], 2);
  // Make sure we cannot share data
  EXPECT_NE(img->image.data, &msg.data[0]);
}
