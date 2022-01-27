/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc,
*  Copyright (c) 2015, Tal Regev.
*  Copyright (c) 2018 Intel Corporation.
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

#ifndef CV_BRIDGE__CV_BRIDGE_H_
#define CV_BRIDGE__CV_BRIDGE_H_

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cv_bridge/cv_bridge_export.h>

#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>

namespace cv_bridge
{

class CV_BRIDGE_EXPORT Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string & description)
  : std::runtime_error(description) {}
};

class CvImage;

typedef std::shared_ptr<CvImage> CvImagePtr;
typedef std::shared_ptr<CvImage const> CvImageConstPtr;

// From: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat
// imread(const string& filename, int flags)
typedef enum
{
  BMP, DIB,
  JPG, JPEG, JPE,
  JP2,
  PNG,
  PBM, PGM, PPM,
  SR, RAS,
  TIFF, TIF,
} Format;

/**
 * \brief Image message class that is interoperable with sensor_msgs/Image but uses a
 * more convenient cv::Mat representation for the image data.
 */
class CV_BRIDGE_EXPORT CvImage
{
public:
  std_msgs::msg::Header header;  // !< ROS header
  std::string encoding;    // !< Image encoding ("mono8", "bgr8", etc.)
  cv::Mat image;           // !< Image data for use with OpenCV

  /**
   * \brief Empty constructor.
   */
  CvImage() {}

  /**
   * \brief Constructor.
   */
  CvImage(
    const std_msgs::msg::Header & header, const std::string & encoding,
    const cv::Mat & image = cv::Mat())
  : header(header), encoding(encoding), image(image)
  {
  }

  /**
   * \brief Convert this message to a ROS sensor_msgs::msg::Image message.
   *
   * The returned sensor_msgs::msg::Image message contains a copy of the image data.
   */
  sensor_msgs::msg::Image::SharedPtr toImageMsg() const;

  /**
   * dst_format is compress the image to desire format.
   * Default value is empty string that will convert to jpg format.
   * can be: jpg, jp2, bmp, png, tif at the moment
   * support this format from opencv:
   * http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)
   */
  sensor_msgs::msg::CompressedImage::SharedPtr toCompressedImageMsg(
    const Format dst_format =
    JPG) const;

  /**
   * \brief Copy the message data to a ROS sensor_msgs::msg::Image message.
   *
   * This overload is intended mainly for aggregate messages such as stereo_msgs::DisparityImage,
   * which contains a sensor_msgs::msg::Image as a data member.
   */
  void toImageMsg(sensor_msgs::msg::Image & ros_image) const;

  /**
   * dst_format is compress the image to desire format.
   * Default value is empty string that will convert to jpg format.
   * can be: jpg, jp2, bmp, png, tif at the moment
   * support this format from opencv:
   * http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)
   */
  void toCompressedImageMsg(
    sensor_msgs::msg::CompressedImage & ros_image,
    const Format dst_format = JPG) const;


  typedef std::shared_ptr<CvImage> Ptr;
  typedef std::shared_ptr<CvImage const> ConstPtr;

protected:
  std::shared_ptr<void const> tracked_object_;  // for sharing ownership

  /// @cond DOXYGEN_IGNORE
  friend
  CV_BRIDGE_EXPORT CvImageConstPtr toCvShare(
    const sensor_msgs::msg::Image & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
};


/**
 * \brief Convert a sensor_msgs::msg::Image message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A shared_ptr to a sensor_msgs::msg::Image message
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
CV_BRIDGE_EXPORT CvImagePtr toCvCopy(
  const sensor_msgs::msg::Image::ConstSharedPtr & source,
  const std::string & encoding = std::string());

CV_BRIDGE_EXPORT CvImagePtr toCvCopy(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert a sensor_msgs::msg::Image message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A sensor_msgs::msg::Image message
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
CV_BRIDGE_EXPORT CvImagePtr toCvCopy(
  const sensor_msgs::msg::Image & source,
  const std::string & encoding = std::string());

CV_BRIDGE_EXPORT CvImagePtr toCvCopy(
  const sensor_msgs::msg::CompressedImage & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::msg::Image message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * \param source   A shared_ptr to a sensor_msgs::msg::Image message
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
CV_BRIDGE_EXPORT CvImageConstPtr toCvShare(
  const sensor_msgs::msg::Image::ConstSharedPtr & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::msg::Image message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * This overload is useful when you have a shared_ptr to a message that contains a
 * sensor_msgs::msg::Image, and wish to share ownership with the containing message.
 *
 * \param source         The sensor_msgs::msg::Image message
 * \param tracked_object A shared_ptr to an object owning the sensor_msgs::msg::Image
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
CV_BRIDGE_EXPORT CvImageConstPtr toCvShare(
  const sensor_msgs::msg::Image & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());

/**
 * \brief Convert a CvImage to another encoding using the same rules as toCvCopy
 */
CV_BRIDGE_EXPORT CvImagePtr cvtColor(
  const CvImageConstPtr & source,
  const std::string & encoding);

struct CvtColorForDisplayOptions
{
  CvtColorForDisplayOptions()
  : do_dynamic_scaling(false),
    min_image_value(0.0),
    max_image_value(0.0),
    colormap(-1),
    bg_label(-1) {}
  bool do_dynamic_scaling;
  double min_image_value;
  double max_image_value;
  int colormap;
  int bg_label;
};


/**
 * \brief Converts an immutable sensor_msgs::msg::Image message to another CvImage for display purposes,
 * using practical conversion rules if needed.
 *
 * Data will be shared between input and output if possible.
 *
 * Recall: sensor_msgs::msg::image_encodings::isColor and isMono tell whether an image contains R,G,B,A, mono
 * (or any combination/subset) with 8 or 16 bit depth.
 *
 * The following rules apply:
 * - if the output encoding is empty, the fact that the input image is mono or multiple-channel is
 * preserved in the ouput image. The bit depth will be 8. it tries to convert to BGR no matter what
 * encoding image is passed.
 * - if the output encoding is not empty, it must have sensor_msgs::msg::image_encodings::isColor and
 * isMono return true. It must also be 8 bit in depth
 * - if the input encoding is an OpenCV format (e.g. 8UC1), and if we have 1,3 or 4 channels, it is
 * respectively converted to mono, BGR or BGRA.
 * - if the input encoding is 32SC1, this estimate that image as label image and will convert it as
 * bgr image with different colors for each label.
 *
 * \param source   A shared_ptr to a sensor_msgs::msg::Image message
 * \param encoding Either an encoding string that returns true in sensor_msgs::msg::image_encodings::isColor
 * isMono or the empty string as explained above.
 * \param options (cv_bridge::CvtColorForDisplayOptions) Options to convert the source image with.
 * - do_dynamic_scaling If true, the image is dynamically scaled between its minimum and maximum value
 * before being converted to its final encoding.
 * - min_image_value Independently from do_dynamic_scaling, if min_image_value and max_image_value are
 * different, the image is scaled between these two values before being converted to its final encoding.
 * - max_image_value Maximum image value
 * - colormap Colormap which the source image converted with.
 */
CV_BRIDGE_EXPORT CvImageConstPtr cvtColorForDisplay(
  const CvImageConstPtr & source,
  const std::string & encoding = std::string(),
  const CvtColorForDisplayOptions options = CvtColorForDisplayOptions());

/**
 * \brief Get the OpenCV type enum corresponding to the encoding.
 *
 * For example, "bgr8" -> CV_8UC3, "32FC1" -> CV_32FC1, and "32FC10" -> CV_32FC10.
 */
CV_BRIDGE_EXPORT int getCvType(const std::string & encoding);

}  // namespace cv_bridge

#endif  // CV_BRIDGE__CV_BRIDGE_H_
