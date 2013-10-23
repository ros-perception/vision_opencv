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

TEST(CvBridgeTest, initialization)
{
  sensor_msgs::Image image;
  cv_bridge::CvImagePtr cv_ptr;

  image.encoding = "bgr8";
  image.height = 200;
  image.width = 200;

  try {
    cv_ptr = cv_bridge::toCvCopy(image, "mono8");
    // Before the fix, it would never get here, as it would segfault
    EXPECT_EQ(1, 0);
  } catch (cv_bridge::Exception& e) {
    EXPECT_EQ(1, 1);
  }

  // Check some normal images with different ratios
  for(int height = 100; height <= 300; ++height) {
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.step = image.width*3;
    image.data.resize(image.height*image.step);
    cv_ptr = cv_bridge::toCvCopy(image, "mono8");
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
