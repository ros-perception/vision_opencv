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

  public:

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

    bool fromImage(const Image& rosimg, std::string encoding = "")
    {
      unsigned int depth;
      if (rosimg.depth == "uint8")
      {
        depth = IPL_DEPTH_8U;
        cvInitImageHeader(rosimg_, cvSize(rosimg.uint8_data.layout.dim[1].size, rosimg.uint8_data.layout.dim[0].size),
                          IPL_DEPTH_8U, rosimg.uint8_data.layout.dim[2].size);
        cvSetData(rosimg_, const_cast<uint8_t*>(&(rosimg.uint8_data.data[0])), rosimg.uint8_data.layout.dim[1].stride);
        img_ = rosimg_;
      } else if (rosimg.depth == "uint16") {
        depth = IPL_DEPTH_16U;
        cvInitImageHeader(rosimg_, cvSize(rosimg.uint16_data.layout.dim[1].size, rosimg.uint16_data.layout.dim[0].size),
                          IPL_DEPTH_16U, rosimg.uint16_data.layout.dim[2].size);
        cvSetData(rosimg_, const_cast<uint16_t*>(&(rosimg.uint16_data.data[0])), rosimg.uint16_data.layout.dim[1].stride*sizeof(uint16_t));
        img_ = rosimg_;
      } else {
        return false;
      }

      if (encoding != "" && (encoding != rosimg.encoding))
      {
        if (encoding == "bgr" && rosimg.encoding == "rgb")
        {
          reallocIfNeeded(&cvtimg_, depth, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_RGB2BGR);
          img_ = cvtimg_;
        }
        else if (encoding == "rgb" && rosimg.encoding == "bgr")
        {
          reallocIfNeeded(&cvtimg_, depth, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_BGR2RGB);
          img_ = cvtimg_;
        }
        else if (encoding == "bgr" && rosimg.encoding == "mono" )
        {
          reallocIfNeeded(&cvtimg_, depth, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_GRAY2BGR);
          img_ = cvtimg_;
        }
        else if (encoding == "mono" && rosimg.encoding == "rgb" )
        {
          reallocIfNeeded(&cvtimg_, depth, 1);
          cvCvtColor(rosimg_, cvtimg_, CV_RGB2GRAY);
          img_ = cvtimg_;
        }
        else if (encoding == "mono" && rosimg.encoding == "bgr" )
        {
          reallocIfNeeded(&cvtimg_, depth, 1);
          cvCvtColor(rosimg_, cvtimg_, CV_BGR2GRAY);
          img_ = cvtimg_;
        }
        else
        {
          return false;
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
     * Converts an openCV IPL Image into a ROS Image that can be sent 'over the wire'.
     * Note that this hasn't been rigorously tested
     * \param source The original Ipl Image that we want to copy from
     * \param dest The ROS Image message that we want to copy to
     */
    static bool fromIpltoRosImage(const IplImage* source, sensor_msgs::Image& dest)
    {
      switch(source->nChannels)
      {
        case 1: dest.encoding = "mono"; break;
        case 3: dest.encoding = "rgb";  break;
        case 4: dest.encoding = "rgba"; break;
        default:
          ROS_ERROR("unknown image format\n");
          return false;
      }
      switch(source->depth)
      {
        case IPL_DEPTH_8U : dest.depth = "uint8";   fillImageHelperCV(dest.uint8_data,   source); break;
        case IPL_DEPTH_8S : dest.depth = "int8";    fillImageHelperCV(dest.int8_data,    source); break;
        case IPL_DEPTH_16U: dest.depth = "uint16";  fillImageHelperCV(dest.uint16_data,  source); break;
        case IPL_DEPTH_16S: dest.depth = "int16";   fillImageHelperCV(dest.int16_data,   source); break;
        case IPL_DEPTH_32S: dest.depth = "int32";   fillImageHelperCV(dest.int32_data,   source); break;
        case IPL_DEPTH_32F: dest.depth = "float32"; fillImageHelperCV(dest.float32_data, source); break;
        case IPL_DEPTH_64F: dest.depth = "float64"; fillImageHelperCV(dest.float64_data, source); break;
        default:
          ROS_ERROR("unsupported depth %d\n", source->depth);
          return false;
      }
      return true;
    }

  private:
    /**
     * Helper method used by fromIplToRosImage in order to populate the images
     */
    template <typename T>
    static void fillImageHelperCV(T& m, const IplImage* frame)
    {
        m.layout.dim.resize(3);
        m.layout.dim.resize(3);
        m.layout.dim[0].label  = "height";
        m.layout.dim[0].size   = frame->height;
        m.layout.dim[0].stride = frame->widthStep*frame->height/sizeof(m.data[0]);
        m.layout.dim[1].label  = "width";
        m.layout.dim[1].size   = frame->width;
        m.layout.dim[1].stride = frame->widthStep/sizeof(m.data[0]);
        m.layout.dim[2].label  = "channel";
        m.layout.dim[2].size   = frame->nChannels;
        m.layout.dim[2].stride = frame->nChannels*sizeof(m.data[0]);
        m.data.resize(frame->widthStep*frame->height/sizeof(m.data[0]));
        memcpy((char*)(&m.data[0]), frame->imageData, m.data.size()*sizeof(m.data[0]));
    }
  };
}


#endif
