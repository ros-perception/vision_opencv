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

#include <string>
#include <vector>
#include <gtest/gtest.h>

#include "opencv2/core/core.hpp"  

#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace sensor_msgs::image_encodings;

bool isUnsigned(const std::string & encoding) {
  return encoding == RGB8 || encoding == RGBA8 || encoding == RGB16 || encoding == RGBA16 || encoding == BGR8 || encoding == BGRA8 || encoding == BGR16 || encoding == BGRA16 || encoding == MONO8 || encoding == MONO16 ||
                           encoding == MONO8 || encoding == MONO16 || encoding == TYPE_8UC1 || encoding == TYPE_8UC2 || encoding == TYPE_8UC3 || encoding == TYPE_8UC4 ||
                           encoding == TYPE_16UC1 || encoding == TYPE_16UC2 || encoding == TYPE_16UC3 || encoding == TYPE_16UC4;
                           //BAYER_RGGB8, BAYER_BGGR8, BAYER_GBRG8, BAYER_GRBG8, BAYER_RGGB16, BAYER_BGGR16, BAYER_GBRG16, BAYER_GRBG16,
                           //YUV422
}
std::vector<std::string>
getEncodings() {
// TODO for Groovy, the following types should be uncommented
std::string encodings[] = { RGB8, RGBA8, RGB16, RGBA16, BGR8, BGRA8, BGR16, BGRA16, MONO8, MONO16,
                           TYPE_8UC1, /*TYPE_8UC2,*/ TYPE_8UC3, TYPE_8UC4,
                           TYPE_8SC1, /*TYPE_8SC2,*/ TYPE_8SC3, TYPE_8SC4,
                           TYPE_16UC1, /*TYPE_16UC2,*/ TYPE_16UC3, TYPE_16UC4,
                           TYPE_16SC1, /*TYPE_16SC2,*/ TYPE_16SC3, TYPE_16SC4,
                           TYPE_32SC1, /*TYPE_32SC2,*/ TYPE_32SC3, TYPE_32SC4,
                           TYPE_32FC1, /*TYPE_32FC2,*/ TYPE_32FC3, TYPE_32FC4,
                           TYPE_64FC1, /*TYPE_64FC2,*/ TYPE_64FC3, TYPE_64FC4,
                           //BAYER_RGGB8, BAYER_BGGR8, BAYER_GBRG8, BAYER_GRBG8, BAYER_RGGB16, BAYER_BGGR16, BAYER_GBRG16, BAYER_GRBG16,
                           YUV422
                         };
return std::vector<std::string>(encodings, encodings+47-8-7);
}

TEST(OpencvTests, testCase_encode_decode)
{
  std::vector<std::string> encodings = getEncodings();
  for(size_t i=0; i<encodings.size(); ++i) {
    std::string encoding1 = encodings[i];
    cv::Mat image_original(cv::Size(400, 400), cv_bridge::getCvType(encoding1));
    cv::RNG r(77);
    r.fill(image_original, cv::RNG::UNIFORM, 0, 255);

    sensor_msgs::Image image_message;
    cv_bridge::CvImage image_bridge(std_msgs::Header(), encoding1, image_original);

    // Convert to a sensor_msgs::Image
    sensor_msgs::ImagePtr image_msg = image_bridge.toImageMsg();

    for(size_t j=0; j<encodings.size(); ++j) {
      std::string encoding2 = encodings[j];
      // It makes sense you cannot convert to a different number of channels and back
      // TODO Actually, that should be a < but we can't with the TYPE_8UC1 conversions: waiting for Groovy
      if (numChannels(encoding1) != numChannels(encoding2)) {
        //EXPECT_THROW(cvtColor(cv_bridge::toCvShare(image_msg, encoding2), encoding1));
        continue;
      }
      // Same if one is a color encoding and not the other one
      // TODO: that should throw but we'll wait for Groovy
      if (isColor(encoding1) != isColor(encoding2))
        continue;
      // Same if they are of different signs
      if ((isUnsigned(encoding1) != isUnsigned(encoding2)) && (!isColor(encoding1)))
        continue;
      // Because of the scaling, we cannot test 16-8 conversion here
      if (bitDepth(encoding1)==16 && bitDepth(encoding2)==8)
        continue;
      // We do not support conversion to YUV422 for now
      if (encoding2 == YUV422)
        continue;
      // We cannot convert to YUV422 so we just perform the first conversion, not the back
      if (encoding1 == YUV422) {
        cv_bridge::toCvShare(image_msg, encoding2);
        continue;
      }

      // And convert back to a cv::Mat
      std::cout << encoding1 << " " << encoding2 << std::endl;
      cv::Mat image_back = cvtColor(cv_bridge::toCvShare(image_msg, encoding2), encoding1)->image;

      EXPECT_LT(cv::norm(image_original, image_back)/image_original.cols/image_original.rows, 0.5) << "problem converting from " << encoding1 << " to " << encoding2 << " and back.";
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
