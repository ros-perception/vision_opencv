#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <gtest/gtest.h>

// Tests conversion of non-continuous cv::Mat. #5206
TEST(CvBridgeTest, NonContinuous)
{
  cv::Mat full = cv::Mat::eye(8, 8, CV_16U);
  cv::Mat partial = full.colRange(2, 5);
  
  cv_bridge::CvImage cvi;
  cvi.encoding = sensor_msgs::image_encodings::MONO16;
  cvi.image = partial;

  sensor_msgs::ImagePtr msg = cvi.toImageMsg();
  EXPECT_EQ(msg->height, 8);
  EXPECT_EQ(msg->width, 3);
  EXPECT_EQ(msg->encoding, cvi.encoding);
  EXPECT_EQ(msg->step, 6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
