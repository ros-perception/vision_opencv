#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <gtest/gtest.h>
#include <cmath>

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
  std::vector<uint8_t> expected = {0, 0, 0, 128, 128, 128, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (unsigned i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(converted->image.at<uint8_t>(i), expected.at(i));
  }
}
