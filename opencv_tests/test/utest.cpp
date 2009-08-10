/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>

#include <opencv/cxtypes.h> 
#include "opencv/cxcore.h"  
#include "opencv/cvwimage.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"


TEST(OpencvTests, testCase_encode_decode)
{
  int fmts[] = { IPL_DEPTH_8U, -1, IPL_DEPTH_8S, IPL_DEPTH_16U, IPL_DEPTH_16S, IPL_DEPTH_32S, IPL_DEPTH_32F, IPL_DEPTH_64F, -1 };

  for (int w = 100; w < 800; w += 100) {
    for (int h = 100; h < 800; h += 100) {
      for (int fi = 0; fmts[fi] != -1; fi++) {
        for (int channels = 1; channels <= 4; channels++) {
          IplImage *original = cvCreateImage(cvSize(w, h), fmts[fi], channels);
#if 0 // XXX OpenCV bug, change this when opencv_latest next moves
          CvRNG r = cvRNG(77);
          cvRandArr(&r, original, CV_RAND_UNI, cvScalar(0,0,0,0), cvScalar(255,255,255,255));
#else
          cvSet(original, cvScalar(1,2,3,4));
#endif

          sensor_msgs::Image image_message;
          sensor_msgs::CvBridge img_bridge_;

          int success;
          success = img_bridge_.fromIpltoRosImage(original, image_message);
          ASSERT_TRUE(success);
          success = img_bridge_.fromImage(image_message);
          ASSERT_TRUE(success);
          IplImage *final = img_bridge_.toIpl();

          // cvSaveImage("final.png", final);
          IplImage *diff = cvCloneImage(original);
          cvAbsDiff(original, final, diff);
          CvScalar sum = cvSum(diff);
          for (int c = 0; c < channels; c++) {
            EXPECT_TRUE(sum.val[c] == 0);
          }
        }
      }
    }
  }
}


TEST(OpencvTests, testCase_decode_8u)
{
  IplImage *original = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
  CvRNG r = cvRNG(77);
  cvRandArr(&r, original, CV_RAND_UNI, cvScalar(0,0,0,0), cvScalar(255,255,255,255));

  sensor_msgs::Image image_message;
  sensor_msgs::CvBridge img_bridge_;

  int success;
  success = img_bridge_.fromIpltoRosImage(original, image_message);
  ASSERT_TRUE(success);

  success = img_bridge_.fromImage(image_message, "passthrough");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 1);

  success = img_bridge_.fromImage(image_message, "mono8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 1);

  success = img_bridge_.fromImage(image_message, "rgb8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 3);

  success = img_bridge_.fromImage(image_message, "bgr8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 3);
}

TEST(OpencvTests, testCase_decode_8uc3)
{
  IplImage *original = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
  CvRNG r = cvRNG(77);
  cvRandArr(&r, original, CV_RAND_UNI, cvScalar(0,0,0,0), cvScalar(255,255,255,255));

  sensor_msgs::Image image_message;
  sensor_msgs::CvBridge img_bridge_;

  int success;
  success = img_bridge_.fromIpltoRosImage(original, image_message);
  ASSERT_TRUE(success);

  success = img_bridge_.fromImage(image_message, "passthrough");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 3);

  success = img_bridge_.fromImage(image_message, "mono8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 1);

  success = img_bridge_.fromImage(image_message, "rgb8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 3);

  success = img_bridge_.fromImage(image_message, "bgr8");
  ASSERT_TRUE(success);
  EXPECT_TRUE(CV_MAT_CN(cvGetElemType(img_bridge_.toIpl())) == 3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
