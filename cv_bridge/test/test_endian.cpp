#include "rcpputils/endian.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <gtest/gtest.h>
#include <memory>

// Byteswap implementations
inline uint32_t byteswap(uint32_t value) noexcept
{
  return ((value & 0x000000FF) << 24) |
         ((value & 0x0000FF00) << 8) |
         ((value & 0x00FF0000) >> 8) |
         ((value & 0xFF000000) >> 24);
}

inline int32_t byteswap(int32_t value) noexcept
{
  return static_cast<int32_t>(byteswap(static_cast<uint32_t>(value)));
}

template <typename T>
inline T native_to_big(T value) noexcept
{
  if constexpr (rcpputils::endian::native == rcpputils::endian::little)
  {
    return byteswap(value);
  }
  return value;
}

template <typename T>
inline T native_to_little(T value) noexcept
{
  if constexpr (rcpputils::endian::native == rcpputils::endian::little)
  {
    return value;
  }
  return byteswap(value);
}

TEST(CvBridgeTest, endianness)
{
  // Create an image of the type opposite to the platform
  sensor_msgs::msg::Image msg;
  msg.height = 1;
  msg.width = 1;
  msg.encoding = "32SC2";
  msg.step = 8;

  msg.data.resize(msg.step);
  int32_t *data = reinterpret_cast<int32_t *>(&msg.data[0]);

  // Write 1 and 2 in order, but with an endianness opposite to the platform
  if (rcpputils::endian::native == rcpputils::endian::little) {
    msg.is_bigendian = true;
    *(data++) = native_to_big(static_cast<int32_t>(1));
    *data = native_to_big(static_cast<int32_t>(2));
  } else {
    msg.is_bigendian = false;
    *(data++) = native_to_little(static_cast<int32_t>(1));
    *data = native_to_little(static_cast<int32_t>(2));
  }

  // Make sure the values are still the same
  cv_bridge::CvImageConstPtr img =
      cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg));
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[0], 1);
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[1], 2);
  // Make sure we cannot share data
  EXPECT_NE(img->image.data, &msg.data[0]);
}
