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
    bool fromImage(const Image& rosimg, std::string encoding = "passthrough")
    {
      CvMat cvmHeader;
      
      int type;
      if (rosimg.encoding == "_8UC1") type = CV_8UC1;
      else if (rosimg.encoding == "_8UC2") type = CV_8UC2;
      else if (rosimg.encoding == "_8UC3") type = CV_8UC3;
      else if (rosimg.encoding == "_8UC4") type = CV_8UC4;
      else if (rosimg.encoding == "_8SC1") type = CV_8SC1;
      else if (rosimg.encoding == "_8SC2") type = CV_8SC2;
      else if (rosimg.encoding == "_8SC3") type = CV_8SC3;
      else if (rosimg.encoding == "_8SC4") type = CV_8SC4;
      else if (rosimg.encoding == "_16UC1") type = CV_16UC1;
      else if (rosimg.encoding == "_16UC2") type = CV_16UC2;
      else if (rosimg.encoding == "_16UC3") type = CV_16UC3;
      else if (rosimg.encoding == "_16UC4") type = CV_16UC4;
      else if (rosimg.encoding == "_16SC1") type = CV_16SC1;
      else if (rosimg.encoding == "_16SC2") type = CV_16SC2;
      else if (rosimg.encoding == "_16SC3") type = CV_16SC3;
      else if (rosimg.encoding == "_16SC4") type = CV_16SC4;
      else if (rosimg.encoding == "_32SC1") type = CV_32SC1;
      else if (rosimg.encoding == "_32SC2") type = CV_32SC2;
      else if (rosimg.encoding == "_32SC3") type = CV_32SC3;
      else if (rosimg.encoding == "_32SC4") type = CV_32SC4;
      else if (rosimg.encoding == "_32FC1") type = CV_32FC1;
      else if (rosimg.encoding == "_32FC2") type = CV_32FC2;
      else if (rosimg.encoding == "_32FC3") type = CV_32FC3;
      else if (rosimg.encoding == "_32FC4") type = CV_32FC4;
      else if (rosimg.encoding == "_64FC1") type = CV_64FC1;
      else if (rosimg.encoding == "_64FC2") type = CV_64FC2;
      else if (rosimg.encoding == "_64FC3") type = CV_64FC3;
      else if (rosimg.encoding == "_64FC4") type = CV_64FC4;
      else if (rosimg.encoding == "rgb8") type = CV_8UC3;
      else if (rosimg.encoding == "bgr8") type = CV_8UC3;
      else if (rosimg.encoding == "rgba8") type = CV_8UC4;
      else if (rosimg.encoding == "bgra8") type = CV_8UC4;
      else if (rosimg.encoding == "mono8") type = CV_8UC1;
      else if (rosimg.encoding == "mono16") type = CV_16UC1;
      else return false;
      cvInitMatHeader(&cvmHeader, rosimg.height, rosimg.width, type, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);
      cvGetImage(&cvmHeader, rosimg_);

      // cvSetData(rosimg_, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);

      if ((encoding == "passthrough") || (rosimg.encoding == encoding)) {
        img_ = rosimg_;
      } else {
        int change = -1;
        int newtype = -1;

        if (rosimg.encoding == "rgb8") {
          if (encoding == "bgr8")
            change = CV_RGB2BGR;
          if (encoding == "bgra8")
            change = CV_RGB2BGRA;
          if (encoding == "rgba8")
            change = CV_RGB2RGBA;
          if (encoding == "mono8" || encoding == "mono16")
            change = CV_RGB2GRAY;
        } else if ((rosimg.encoding == "bgr8") || (rosimg.encoding == "_8UC3")) {
          if (encoding == "rgb8")
            change = CV_BGR2RGB;
          if (encoding == "bgra8")
            change = CV_BGR2BGRA;
          if (encoding == "mono8" || encoding == "mono16")
            change = CV_BGR2GRAY;
        } else if (rosimg.encoding == "rgba8") {
          if (encoding == "rgb8")
            change = CV_RGBA2RGB;
          if (encoding == "bgr8")
            change = CV_RGBA2BGR;
          if (encoding == "mono8" || encoding == "mono16")
            change = CV_RGBA2GRAY;
        } else if ((rosimg.encoding == "bgra8") || (rosimg.encoding == "_8UC4")) {
          if (encoding == "rgb8")
            change = CV_BGRA2RGB;
          if (encoding == "bgr8")
            change = CV_BGRA2BGR;
          if (encoding == "mono8" || encoding == "mono16")
            change = CV_BGRA2GRAY;
        } else if (rosimg.encoding == "mono8" || rosimg.encoding == "mono16"  || rosimg.encoding == "_8UC1") {
          if (encoding == "rgb8")
            change = CV_GRAY2RGB;
          if (encoding == "bgr8")
            change = CV_GRAY2BGR;
          if (encoding == "bgra8")
            change = CV_GRAY2BGRA;
          if (encoding == "rgba8")
            change = CV_GRAY2RGBA;
          if (encoding == "mono8" || encoding == "mono16")
            change = CV_COLORCVT_MAX; // means do no conversion
        }

        if (encoding == "bgr8")
           newtype = CV_8UC3;
        if (encoding == "rgb8")
           newtype = CV_8UC3;
        if (encoding == "bgra8")
           newtype = CV_8UC4;
        if (encoding == "rgba")
           newtype = CV_8UC4;
        if (encoding == "mono8")
           newtype = CV_8UC1;
        if (encoding == "mono16")
           newtype = CV_16UC1;

        if (change == -1 || newtype == -1)
          return false;
        reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, CV_MAT_CN(newtype));
        if (change == CV_COLORCVT_MAX)
          cvConvertScale(rosimg_, cvtimg_);
        else
          cvCvtColor(rosimg_, cvtimg_, change);
        img_ = cvtimg_;
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
    static bool fromIpltoRosImage(const IplImage* source, sensor_msgs::Image& dest, std::string encoding = "passthrough")
    {
      CvMat header, *cvm;

      cvm = cvGetMat(source, &header);
      // dest.type = cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK);
      dest.encoding = encoding;

      if (encoding == "passthrough") {
        switch (cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK)) {
        case CV_8UC1: dest.encoding = "_8UC1"; break;
        case CV_8UC2: dest.encoding = "_8UC2"; break;
        case CV_8UC3: dest.encoding = "_8UC3"; break;
        case CV_8UC4: dest.encoding = "_8UC4"; break;
        case CV_8SC1: dest.encoding = "_8SC1"; break;
        case CV_8SC2: dest.encoding = "_8SC2"; break;
        case CV_8SC3: dest.encoding = "_8SC3"; break;
        case CV_8SC4: dest.encoding = "_8SC4"; break;
        case CV_16UC1: dest.encoding = "_16UC1"; break;
        case CV_16UC2: dest.encoding = "_16UC2"; break;
        case CV_16UC3: dest.encoding = "_16UC3"; break;
        case CV_16UC4: dest.encoding = "_16UC4"; break;
        case CV_16SC1: dest.encoding = "_16SC1"; break;
        case CV_16SC2: dest.encoding = "_16SC2"; break;
        case CV_16SC3: dest.encoding = "_16SC3"; break;
        case CV_16SC4: dest.encoding = "_16SC4"; break;
        case CV_32SC1: dest.encoding = "_32SC1"; break;
        case CV_32SC2: dest.encoding = "_32SC2"; break;
        case CV_32SC3: dest.encoding = "_32SC3"; break;
        case CV_32SC4: dest.encoding = "_32SC4"; break;
        case CV_32FC1: dest.encoding = "_32FC1"; break;
        case CV_32FC2: dest.encoding = "_32FC2"; break;
        case CV_32FC3: dest.encoding = "_32FC3"; break;
        case CV_32FC4: dest.encoding = "_32FC4"; break;
        case CV_64FC1: dest.encoding = "_64FC1"; break;
        case CV_64FC2: dest.encoding = "_64FC2"; break;
        case CV_64FC3: dest.encoding = "_64FC3"; break;
        case CV_64FC4: dest.encoding = "_64FC4"; break;
        default: assert(0);
        }
      } else {
        // XXX JCB - should verify encoding
        // XXX JCB - should verify that channels match the channels in original image
        dest.encoding = encoding;
      }

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
