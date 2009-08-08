/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef CVBRIDGE_HH
#define CVBRIDGE_HH

#include "sensor_msgs/Image.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"

namespace sensor_msgs
{

  class CvBridge
  {
  public:

    IplImage* img_;
    IplImage* rosimg_;
    IplImage* cvtimg_;

    bool reallocIfNeeded_(IplImage** img, CvSize sz, int depth, int channels)
    {
      if ((*img) != 0)
        if ((*img)->width     != sz.width  ||
            (*img)->height    != sz.height ||
            (*img)->depth     != depth     ||
            (*img)->nChannels != channels)
        {
          cvReleaseImage(img);
          *img = 0;
        }

      if (*img == 0)
      {
        *img = cvCreateImage(sz, depth, channels);
        return true;
      }
      return false;
    }

    CvBridge() : img_(0), rosimg_(0), cvtimg_(0)
    {
      rosimg_ = cvCreateImageHeader( cvSize(0,0), IPL_DEPTH_8U, 1 );
    }

    ~CvBridge()
    {
      if (rosimg_) {
        cvReleaseImageHeader(&rosimg_);
	rosimg_ = 0;
      }

      if (cvtimg_) {
        cvReleaseImage(&cvtimg_);
	cvtimg_ = 0;
      }
    }

    inline IplImage* toIpl()
    {
      return img_;
    }

    /**
     * Converts a ROS Image into an OpenCV IPL Image.
     * \param rosimg The ROS Image message
     */
    bool fromImage(const Image& rosimg, std::string encoding = "")
    {
      CvMat cvmHeader;
      
      cvInitMatHeader(&cvmHeader, rosimg.height, rosimg.width, rosimg.type, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);
      cvGetImage(&cvmHeader, rosimg_);

      // cvSetData(rosimg_, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);

      if (encoding == "") {
        img_ = rosimg_;
      } else {
        if ((encoding == "bgr" || encoding == "rgb") && rosimg.type == CV_8UC1)
        {
          reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_GRAY2BGR);
          img_ = cvtimg_;
        }
        else if (encoding == "mono" && rosimg.type == CV_8UC3)
        {
          reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, 1);
          cvCvtColor(rosimg_, cvtimg_, CV_RGB2GRAY);
          img_ = cvtimg_;
        } else {
          img_ = rosimg_;
        }
      }
      return true;
    }

    bool reallocIfNeeded(IplImage** img, int depth = -1, int channels = -1)
    {
      if (depth == -1)
        depth = img_->depth;
      if (channels == -1)
        channels = img_->nChannels;
      return reallocIfNeeded_(img, cvGetSize(img_), depth, channels);
    }

    /**
     * Converts an OpenCV IPL Image into a ROS Image that can be sent 'over the wire'.
     * \param source The original Ipl Image that we want to copy from
     * \param dest The ROS Image message that we want to copy to
     */
    static bool fromIpltoRosImage(const IplImage* source, sensor_msgs::Image& dest)
    {
      CvMat header, *cvm;

      cvm = cvGetMat(source, &header);
      dest.type = cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK);
      dest.width = cvm->width;
      dest.height = cvm->height;
      dest.step = cvm->step;
      dest.data.resize(cvm->step * cvm->height);
      memcpy((char*)(&dest.data[0]), source->imageData, cvm->step * cvm->height);
      return true;
    }
  };
}


#endif
