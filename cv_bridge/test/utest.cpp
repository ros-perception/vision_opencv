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

TEST(CvBridgeTest, imageMessageStep)
{
  // Test 1: image step is padded
  sensor_msgs::Image image;
  cv_bridge::CvImagePtr cv_ptr;

  image.encoding = "mono8";
  image.height = 220;
  image.width = 200;
  image.is_bigendian = false;
  image.step = 208;

  image.data.resize(image.height*image.step);

  ASSERT_NO_THROW(cv_ptr = cv_bridge::toCvCopy(image, "mono8"));
  ASSERT_EQ(220, cv_ptr->image.rows);
  ASSERT_EQ(200, cv_ptr->image.cols);
  //OpenCV copyTo argument removes the stride
  ASSERT_EQ(200, cv_ptr->image.step[0]);

  //Test 2: image step is invalid
  image.step = 199;

  ASSERT_THROW(cv_ptr = cv_bridge::toCvCopy(image, "mono8"), cv_bridge::Exception);

  //Test 3: image step == image.width * element size * number of channels
  image.step = 200;
  image.data.resize(image.height*image.step);

  ASSERT_NO_THROW(cv_ptr = cv_bridge::toCvCopy(image, "mono8"));
  ASSERT_EQ(220, cv_ptr->image.rows);
  ASSERT_EQ(200, cv_ptr->image.cols);
  ASSERT_EQ(200, cv_ptr->image.step[0]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
