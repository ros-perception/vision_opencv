/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef CV_BRIDGE_CV_BRIDGE_H
#define CV_BRIDGE_CV_BRIDGE_H

#include <sensor_msgs/Image.h>
#include <ros/static_assert.h>
#include <opencv2/core/core.hpp>
#include <stdexcept>

namespace cv_bridge {

class Exception : public std::runtime_error
{
public:
  Exception(const std::string& description) : std::runtime_error(description) {}
};

class CvImage;

typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

/**
 * \brief Image message class that is interoperable with sensor_msgs/Image but uses a
 * more convenient cv::Mat representation for the image data.
 */
class CvImage
{
public:
  std_msgs::Header header; //!< ROS header
  std::string encoding;    //!< Image encoding ("mono8", "bgr8", etc.)
  cv::Mat image;           //!< Image data for use with OpenCV

  /**
   * \brief Empty constructor.
   */
  CvImage() {}

  /**
   * \brief Constructor.
   */
  CvImage(const std_msgs::Header& header, const std::string& encoding,
          const cv::Mat& image = cv::Mat())
    : header(header), encoding(encoding), image(image)
  {
  }
  
  /**
   * \brief Convert this message to a ROS sensor_msgs::Image message.
   *
   * The returned sensor_msgs::Image message contains a copy of the image data.
   */
  sensor_msgs::ImagePtr toImageMsg() const;

  /**
   * \brief Copy the message data to a ROS sensor_msgs::Image message.
   *
   * This overload is intended mainly for aggregate messages such as stereo_msgs::DisparityImage,
   * which contains a sensor_msgs::Image as a data member.
   */
  void toImageMsg(sensor_msgs::Image& ros_image) const;

  typedef boost::shared_ptr<CvImage> Ptr;
  typedef boost::shared_ptr<CvImage const> ConstPtr;

protected:
  boost::shared_ptr<void const> tracked_object_; // for sharing ownership

  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                            const boost::shared_ptr<void const>& tracked_object,
                            const std::string& encoding);
  /// @endcond
};

/**
 * \brief Convert a sensor_msgs::Image message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A shared_ptr to a sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                    const std::string& encoding = std::string());

/**
 * \brief Convert a sensor_msgs::Image message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 * If the source is 8bit and the encoding 16 or vice-versa, a scaling is applied (65535/255 and
 * 255/65535 respectively). Otherwise, no scaling is applied and the rules from the convertTo OpenCV
 * function are applied (capping): http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-convertto
 */
CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                    const std::string& encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::Image message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * \param source   A shared_ptr to a sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                          const std::string& encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::Image message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * This overload is useful when you have a shared_ptr to a message that contains a
 * sensor_msgs::Image, and wish to share ownership with the containing message.
 *
 * \param source         The sensor_msgs::Image message
 * \param tracked_object A shared_ptr to an object owning the sensor_msgs::Image
 * \param encoding       The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                          const boost::shared_ptr<void const>& tracked_object,
                          const std::string& encoding = std::string());

/**
 * \brief Convert a CvImage to another encoding using the same rules as toCvCopy
 */
CvImagePtr cvtColor(const CvImageConstPtr& source,
                    const std::string& encoding);

/**
 * \brief Get the OpenCV type enum corresponding to the encoding.
 *
 * For example, "bgr8" -> CV_8UC3.
 */
int getCvType(const std::string& encoding);

} // namespace cv_bridge



// CvImage as a first class message type

// The rest of this file hooks into the roscpp serialization API to make CvImage
// a first-class message type you can publish and subscribe to directly.
// Unfortunately this doesn't yet work with image_transport, so don't rewrite all
// your callbacks to use CvImage! It might be useful for specific tasks, like
// processing bag files.

/// @cond DOXYGEN_IGNORE
namespace ros {

namespace message_traits {

template<> struct MD5Sum<cv_bridge::CvImage>
{
  static const char* value() { return MD5Sum<sensor_msgs::Image>::value(); }
  static const char* value(const cv_bridge::CvImage&) { return value(); }

  static const uint64_t static_value1 = MD5Sum<sensor_msgs::Image>::static_value1;
  static const uint64_t static_value2 = MD5Sum<sensor_msgs::Image>::static_value2;
  
  // If the definition of sensor_msgs/Image changes, we'll get a compile error here.
  ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::Image>::static_value1 == 0x060021388200f6f0ULL);
  ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::Image>::static_value2 == 0xf447d0fcd9c64743ULL);
};

template<> struct DataType<cv_bridge::CvImage>
{
  static const char* value() { return DataType<sensor_msgs::Image>::value(); }
  static const char* value(const cv_bridge::CvImage&) { return value(); }
};

template<> struct Definition<cv_bridge::CvImage>
{
  static const char* value() { return Definition<sensor_msgs::Image>::value(); }
  static const char* value(const cv_bridge::CvImage&) { return value(); }
};

template<> struct HasHeader<cv_bridge::CvImage> : TrueType {};

} // namespace ros::message_traits

namespace serialization {

template<> struct Serializer<cv_bridge::CvImage>
{
  /// @todo Still ignoring endianness...
  
  template<typename Stream>
  inline static void write(Stream& stream, const cv_bridge::CvImage& m)
  {
    stream.next(m.header);
    stream.next((uint32_t)m.image.rows); // height
    stream.next((uint32_t)m.image.cols); // width
    stream.next(m.encoding);
    uint8_t is_bigendian = 0;
    stream.next(is_bigendian);
    stream.next((uint32_t)m.image.step);
    size_t data_size = m.image.step*m.image.rows;
    stream.next((uint32_t)data_size);
    if (data_size > 0)
      memcpy(stream.advance(data_size), m.image.data, data_size);
  }

  template<typename Stream>
  inline static void read(Stream& stream, cv_bridge::CvImage& m)
  {
    stream.next(m.header);
    uint32_t height, width;
    stream.next(height);
    stream.next(width);
    stream.next(m.encoding);
    uint8_t is_bigendian;
    stream.next(is_bigendian);
    uint32_t step, data_size;
    stream.next(step);
    stream.next(data_size);
    int type = cv_bridge::getCvType(m.encoding);
    // Construct matrix pointing to the stream data, then copy it to m.image
    cv::Mat tmp((int)height, (int)width, type, stream.advance(data_size), (size_t)step);
    tmp.copyTo(m.image);
  }

  inline static uint32_t serializedLength(const cv_bridge::CvImage& m)
  {
    size_t data_size = m.image.step*m.image.rows;
    return serializationLength(m.header) + serializationLength(m.encoding) + 17 + data_size;
  }
};

} // namespace ros::serialization

namespace message_operations {

template<> struct Printer<cv_bridge::CvImage>
{
  template<typename Stream>
  static void stream(Stream& s, const std::string& indent, const cv_bridge::CvImage& m)
  {
    /// @todo Replicate printing for sensor_msgs::Image
  }
};

} // namespace ros::message_operations

} // namespace ros

namespace cv_bridge {

inline std::ostream& operator<<(std::ostream& s, const CvImage& m)
{
  ros::message_operations::Printer<CvImage>::stream(s, "", m);
  return s;
}

} // namespace cv_bridge

/// @endcond

#endif
