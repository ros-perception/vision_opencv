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

#ifndef CVBRIDGE_HH
#define CVBRIDGE_HH

#include <stdexcept>
#include "sensor_msgs/Image.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"

namespace sensor_msgs
{
  class CvBridgeException: public std::runtime_error
  {
  public:
    CvBridgeException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
  };

  /**
   * \brief Old C++ API. Use cv_bridge::CvImage, toCvCopy(), toCvShare() instead.
   */
  class CvBridge
  {
  public:

    // @cond DOXYGEN_IGNORE

    IplImage* img_;
    IplImage* rosimg_;
    IplImage* cvtimg_;

    ROS_DEPRECATED CvBridge() : img_(0), rosimg_(0), cvtimg_(0)
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

    bool reallocIfNeeded(IplImage** img, int depth = -1, int channels = -1)
    {
      if (depth == -1)
        depth = img_->depth;
      if (channels == -1)
        channels = img_->nChannels;
      return reallocIfNeeded_(img, cvGetSize(img_), depth, channels);
    }

    int encoding_as_cvtype(std::string encoding)
    {
      if (encoding == "8UC1") return CV_8UC1;
      if (encoding == "8UC2") return CV_8UC2;
      if (encoding == "8UC3") return CV_8UC3;
      if (encoding == "8UC4") return CV_8UC4;
      if (encoding == "8SC1") return CV_8SC1;
      if (encoding == "8SC2") return CV_8SC2;
      if (encoding == "8SC3") return CV_8SC3;
      if (encoding == "8SC4") return CV_8SC4;
      if (encoding == "16UC1") return CV_16UC1;
      if (encoding == "16UC2") return CV_16UC2;
      if (encoding == "16UC3") return CV_16UC3;
      if (encoding == "16UC4") return CV_16UC4;
      if (encoding == "16SC1") return CV_16SC1;
      if (encoding == "16SC2") return CV_16SC2;
      if (encoding == "16SC3") return CV_16SC3;
      if (encoding == "16SC4") return CV_16SC4;
      if (encoding == "32SC1") return CV_32SC1;
      if (encoding == "32SC2") return CV_32SC2;
      if (encoding == "32SC3") return CV_32SC3;
      if (encoding == "32SC4") return CV_32SC4;
      if (encoding == "32FC1") return CV_32FC1;
      if (encoding == "32FC2") return CV_32FC2;
      if (encoding == "32FC3") return CV_32FC3;
      if (encoding == "32FC4") return CV_32FC4;
      if (encoding == "64FC1") return CV_64FC1;
      if (encoding == "64FC2") return CV_64FC2;
      if (encoding == "64FC3") return CV_64FC3;
      if (encoding == "64FC4") return CV_64FC4;
      if (encoding == "rgb8") return CV_8UC3;
      if (encoding == "bgr8") return CV_8UC3;
      if (encoding == "rgba8") return CV_8UC4;
      if (encoding == "bgra8") return CV_8UC4;
      if (encoding == "mono8") return CV_8UC1;
      if (encoding == "mono16") return CV_16UC1;
      return -1;
    }

    std::string encoding_as_fmt(std::string encoding)
    {
      std::string fmt;
      int source_channels = CV_MAT_CN(encoding_as_cvtype(encoding));
      if (source_channels == -1)
        fmt == "";
      else if (source_channels == 1)
        fmt = "GRAY";
      else if ("rgb8" == encoding)
        fmt = "RGB";
      else if ("rgba8" == encoding)
        fmt = "RGBA";
      else if (source_channels == 3)
        fmt = "BGR";
      else if (source_channels == 4)
        fmt = "BGRA";
      return fmt;
    }
    // @endcond


    /**
     * Returns the OpenCV IPL Image created by method fromImage.
     *
     * This method will be deprecated and removed in a future release.  Use method imgMsgToCv instead.
     *
     */
    inline IplImage* toIpl()
    {
      return img_;
    }

    /**
     * Converts a ROS Image into an OpenCV IPL Image.
     *
     * This method will be deprecated and removed in a future release.  Use method imgMsgToCv instead.
     *
     * \param rosimg            The ROS Image message
     * \param desired_encoding  image encoding.  See method imgMsgToCv for details.
     */
    bool fromImage(const Image& rosimg, std::string desired_encoding = "passthrough")
    {
      CvMat cvmHeader;
      
      // cvSetData(rosimg_, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);

      int source_type = encoding_as_cvtype(rosimg.encoding);

      cvInitMatHeader(&cvmHeader, rosimg.height, rosimg.width, source_type, const_cast<uint8_t*>(&(rosimg.data[0])), rosimg.step);
      cvGetImage(&cvmHeader, rosimg_);

      // Check that the message encoding is legal
      if (encoding_as_cvtype(rosimg.encoding) == -1)
        return false;

      if (desired_encoding == "passthrough") {
        img_ = rosimg_;
      } else {
        // Might need to do a conversion.  sourcefmt and destfmt can be
        // one of GRAY, RGB, BGR, RGBA, BGRA.
        std::string sourcefmt = encoding_as_fmt(rosimg.encoding);
        std::string destfmt = encoding_as_fmt(desired_encoding);
        int destination_type = encoding_as_cvtype(desired_encoding);

        if ((sourcefmt == destfmt) && (source_type == destination_type)) {
          img_ = rosimg_;
        } else {
          img_ = rosimg_;  // realloc uses this as a hidden argument.
          if (desired_encoding != "mono16")
            reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, CV_MAT_CN(destination_type));
          else
            reallocIfNeeded(&cvtimg_, IPL_DEPTH_16U, CV_MAT_CN(destination_type));
          void *sourceimg;
          CvMat *same_depth = NULL;
          if ((source_type & CV_MAT_DEPTH_MASK) != (destination_type & CV_MAT_DEPTH_MASK)) {
            // need to convert depth, but not number of channels, which happens below
            same_depth = cvCreateMat(rosimg.height,
                rosimg.width,
                CV_MAKETYPE((destination_type & CV_MAT_TYPE_MASK), CV_MAT_CN(source_type)));
            cvConvertScale(rosimg_, same_depth);
            sourceimg = same_depth;
          } else {
            sourceimg = rosimg_;
          }
          if (sourcefmt == destfmt) {
            cvConvertScale(sourceimg, cvtimg_);
          } else {
            if (sourcefmt == "") {
              return false;
            }
            if (sourcefmt == "GRAY") {
              if (destfmt == "RGB")
                cvCvtColor(sourceimg, cvtimg_, CV_GRAY2RGB);
              if (destfmt == "BGR")
                cvCvtColor(sourceimg, cvtimg_, CV_GRAY2BGR);
              if (destfmt == "RGBA")
                cvCvtColor(sourceimg, cvtimg_, CV_GRAY2RGBA);
              if (destfmt == "BGRA")
                cvCvtColor(sourceimg, cvtimg_, CV_GRAY2BGRA);
            }
            if (sourcefmt == "RGB") {
              if (destfmt == "GRAY")
                cvCvtColor(sourceimg, cvtimg_, CV_RGB2GRAY);
              if (destfmt == "BGR")
                cvCvtColor(sourceimg, cvtimg_, CV_RGB2BGR);
              if (destfmt == "RGBA")
                cvCvtColor(sourceimg, cvtimg_, CV_RGB2RGBA);
              if (destfmt == "BGRA")
                cvCvtColor(sourceimg, cvtimg_, CV_RGB2BGRA);
            }
            if (sourcefmt == "BGR") {
              if (destfmt == "GRAY")
                cvCvtColor(sourceimg, cvtimg_, CV_BGR2GRAY);
              if (destfmt == "RGB")
                cvCvtColor(sourceimg, cvtimg_, CV_BGR2RGB);
              if (destfmt == "RGBA")
                cvCvtColor(sourceimg, cvtimg_, CV_BGR2RGBA);
              if (destfmt == "BGRA")
                cvCvtColor(sourceimg, cvtimg_, CV_BGR2BGRA);
            }
            if (sourcefmt == "RGBA") {
              if (destfmt == "GRAY")
                cvCvtColor(sourceimg, cvtimg_, CV_RGBA2GRAY);
              if (destfmt == "RGB")
                cvCvtColor(sourceimg, cvtimg_, CV_RGBA2RGB);
              if (destfmt == "BGR")
                cvCvtColor(sourceimg, cvtimg_, CV_RGBA2BGR);
              if (destfmt == "BGRA")
                cvCvtColor(sourceimg, cvtimg_, CV_RGBA2BGRA);
            }
            if (sourcefmt == "BGRA") {
              if (destfmt == "GRAY")
                cvCvtColor(sourceimg, cvtimg_, CV_BGRA2GRAY);
              if (destfmt == "RGB")
                cvCvtColor(sourceimg, cvtimg_, CV_BGRA2RGB);
              if (destfmt == "BGR")
                cvCvtColor(sourceimg, cvtimg_, CV_BGRA2BGR);
              if (destfmt == "RGBA")
                cvCvtColor(sourceimg, cvtimg_, CV_BGRA2RGBA);
            }
          }
          if (same_depth != NULL) {
              cvReleaseMat(&same_depth);
          }
          img_ = cvtimg_;
        }
      }
      return true;
    }

    /**
     * Converts an OpenCV IPL Image into a ROS Image that can be sent 'over the wire'.
     *
     * \param source    The original Ipl Image that we want to copy from
     * \param dest      The ROS Image message that we want to copy to
     * \param encoding  image encoding.  See method cvToImgMsg for details.
     */
    ROS_DEPRECATED static bool fromIpltoRosImage(const IplImage* source, sensor_msgs::Image& dest, std::string encoding = "passthrough")
    {
      CvMat header, *cvm;

      cvm = cvGetMat(source, &header);
      // dest.type = cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK);
      dest.encoding = encoding;

      if (encoding == "passthrough") {
        switch (cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK)) {
        case CV_8UC1: dest.encoding = "8UC1"; break;
        case CV_8UC2: dest.encoding = "8UC2"; break;
        case CV_8UC3: dest.encoding = "8UC3"; break;
        case CV_8UC4: dest.encoding = "8UC4"; break;
        case CV_8SC1: dest.encoding = "8SC1"; break;
        case CV_8SC2: dest.encoding = "8SC2"; break;
        case CV_8SC3: dest.encoding = "8SC3"; break;
        case CV_8SC4: dest.encoding = "8SC4"; break;
        case CV_16UC1: dest.encoding = "16UC1"; break;
        case CV_16UC2: dest.encoding = "16UC2"; break;
        case CV_16UC3: dest.encoding = "16UC3"; break;
        case CV_16UC4: dest.encoding = "16UC4"; break;
        case CV_16SC1: dest.encoding = "16SC1"; break;
        case CV_16SC2: dest.encoding = "16SC2"; break;
        case CV_16SC3: dest.encoding = "16SC3"; break;
        case CV_16SC4: dest.encoding = "16SC4"; break;
        case CV_32SC1: dest.encoding = "32SC1"; break;
        case CV_32SC2: dest.encoding = "32SC2"; break;
        case CV_32SC3: dest.encoding = "32SC3"; break;
        case CV_32SC4: dest.encoding = "32SC4"; break;
        case CV_32FC1: dest.encoding = "32FC1"; break;
        case CV_32FC2: dest.encoding = "32FC2"; break;
        case CV_32FC3: dest.encoding = "32FC3"; break;
        case CV_32FC4: dest.encoding = "32FC4"; break;
        case CV_64FC1: dest.encoding = "64FC1"; break;
        case CV_64FC2: dest.encoding = "64FC2"; break;
        case CV_64FC3: dest.encoding = "64FC3"; break;
        case CV_64FC4: dest.encoding = "64FC4"; break;
        default: assert(0);
        }
      } else {
        int ct = cvm->type & (CV_MAT_TYPE_MASK | CV_MAT_DEPTH_MASK);
        if (encoding == "rgb8") {
          if (ct != CV_8UC3)
            return false;
        } else if (encoding == "rgba8") {
          if (ct != CV_8UC4)
            return false;
        } else if (encoding == "bgr8") {
          if (ct != CV_8UC3)
            return false;
        } else if (encoding == "bgra8") {
          if (ct != CV_8UC4)
            return false;
        } else if (encoding == "mono8") {
          if (ct != CV_8UC1)
            return false;
        } else if (encoding == "mono16") {
          if (ct != CV_16UC1)
            return false;
        } else {
          return false;
        }

        dest.encoding = encoding;
      }

      dest.width = cvm->width;
      dest.height = cvm->height;
      dest.step = cvm->step;
      dest.data.resize(cvm->step * cvm->height);
      memcpy((char*)(&dest.data[0]), source->imageData, cvm->step * cvm->height);
      return true;
    }

    /**
     * 
        Convert an OpenCV CvArr type (that is, an IplImage or CvMat) to a ROS sensor_msgs::Image message.
        \param source      An OpenCV IplImage or CvMat
        \param encoding  The encoding of the image data, one of the following strings:
           - \c "passthrough"
           - \c "rgb8"
           - \c "rgba8"
           - \c "bgr8"
           - \c "bgra8"
           - \c "mono8"
           - \c "mono16"
            
        If \a encoding is \c "passthrough", then the message has the same encoding as the image's OpenCV type.
        Otherwise \a encoding must be one of the strings \c "rgb8", \c "bgr8", \c "rgba8", \c "bgra8", \c "mono8" or \c "mono16",
        in which case the OpenCV image must have the appropriate type:
           - \c CV_8UC3 (for \c "rgb8", \c "bgr8"),
           - \c CV_8UC4 (for \c "rgba8", \c "bgra8"),
           - \c CV_8UC1 (for \c "mono8"), or
           - \c CV_16UC1 (for \c "mono16")

        This function returns a sensor_msgs::Image message on success, or raises CvBridgeException on failure.
     */
    ROS_DEPRECATED static sensor_msgs::Image::Ptr cvToImgMsg(const IplImage* source, std::string encoding = "passthrough")
    {
      sensor_msgs::Image::Ptr rosimg(new sensor_msgs::Image);
      if (!fromIpltoRosImage(source, *rosimg, encoding))
        throw CvBridgeException("Conversion to OpenCV image failed");
      return rosimg;
    }

    /**
        Convert a
        sensor_msgs::Image message to an OpenCV IplImage.

        \param rosimg   A sensor_msgs::Image message
        \param desired_encoding  The encoding of the image data, one of the following strings:
           - \c "passthrough"
           - \c "rgb8"
           - \c "rgba8"
           - \c "bgr8"
           - \c "bgra8"
           - \c "mono8"
           - \c "mono16"
            
        If \a desired_encoding is \c "passthrough", then the returned image has the same format as \a img_msg.
        Otherwise \a desired_encoding must be one of the strings \c "rgb8", \c "bgr8", \c "rgba8", \c "bgra8", \c "mono8" or \c "mono16",
        in which case this method converts the image using
        \c cvCvtColor (http://opencv.willowgarage.com/documentation/image_processing.html#cvCvtColor)
        (if necessary) and the returned image has a type as follows:
           - \c CV_8UC3 (for \c "rgb8", \c "bgr8"),
           - \c CV_8UC4 (for \c "rgba8", \c "bgra8"),
           - \c CV_8UC1 (for \c "mono8"), or
           - \c CV_16UC1 (for \c "mono16")

        This function returns an OpenCV IplImage on success, or raises CvBridgeException on failure.

     */
    IplImage* imgMsgToCv(sensor_msgs::Image::ConstPtr rosimg, std::string desired_encoding = "passthrough")
    {
      if (!fromImage(*rosimg, desired_encoding))
        throw CvBridgeException("Conversion to OpenCV image failed");
      return toIpl();
    }
  };
}

#endif
