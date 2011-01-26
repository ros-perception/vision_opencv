#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/make_shared.hpp>

namespace enc = sensor_msgs::image_encodings;

/// @todo Handle endianness - e.g. 16-bit dc1394 camera images are big-endian
namespace cv_bridge {

int getCvType(const std::string& encoding)
{
  // Check for the most common encodings first
  if (encoding == enc::BGR8)   return CV_8UC3;
  if (encoding == enc::MONO8)  return CV_8UC1;
  if (encoding == enc::RGB8)   return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGRA8)  return CV_8UC4;
  if (encoding == enc::RGBA8)  return CV_8UC4;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;

  // Check all the generic content encodings
#define CHECK_ENCODING(code)                            \
  if (encoding == enc::TYPE_##code) return CV_##code    \
  /***/
#define CHECK_CHANNEL_TYPE(t)                   \
  CHECK_ENCODING(t##1);                         \
  CHECK_ENCODING(t##2);                         \
  CHECK_ENCODING(t##3);                         \
  CHECK_ENCODING(t##4);                         \
  /***/

  CHECK_CHANNEL_TYPE(8UC);
  CHECK_CHANNEL_TYPE(8SC);
  CHECK_CHANNEL_TYPE(16UC);
  CHECK_CHANNEL_TYPE(16SC);
  CHECK_CHANNEL_TYPE(32SC);
  CHECK_CHANNEL_TYPE(32FC);
  CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

  throw Exception("Unrecognized image encoding [" + encoding + "]");
}

/// @cond DOXYGEN_IGNORE

enum Format { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA };

Format getFormat(const std::string& encoding, int cv_type)
{
  if (encoding == enc::BGR8)   return BGR;
  if (encoding == enc::MONO8)  return GRAY;
  if (encoding == enc::RGB8)   return RGB;
  if (encoding == enc::MONO16) return GRAY;
  if (encoding == enc::BGRA8)  return BGRA;
  if (encoding == enc::RGBA8)  return RGBA;

  // We don't support conversions to/from other types
  return INVALID;
}

static const int SAME_FORMAT = -1;

int getConversionCode(Format src_format, Format dst_format)
{
  static const int CONVERSION_CODES[] = { SAME_FORMAT,
                                          CV_GRAY2RGB,
                                          CV_GRAY2BGR,
                                          CV_GRAY2RGBA,
                                          CV_GRAY2BGRA,
                                          CV_RGB2GRAY,
                                          SAME_FORMAT,
                                          CV_RGB2BGR,
                                          CV_RGB2RGBA,
                                          CV_RGB2BGRA,
                                          CV_BGR2GRAY,
                                          CV_BGR2RGB,
                                          SAME_FORMAT,
                                          CV_BGR2RGBA,
                                          CV_BGR2BGRA,
                                          CV_RGBA2GRAY,
                                          CV_RGBA2RGB,
                                          CV_RGBA2BGR,
                                          SAME_FORMAT,
                                          CV_RGBA2BGRA,
                                          CV_BGRA2GRAY,
                                          CV_BGRA2RGB,
                                          CV_BGRA2BGR,
                                          CV_BGRA2RGBA,
                                          SAME_FORMAT };
  return CONVERSION_CODES[src_format*5 + dst_format];
}

/// @endcond

sensor_msgs::ImagePtr CvImage::toImageMsg() const
{
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  toImageMsg(*ptr);
  return ptr;
}

void CvImage::toImageMsg(sensor_msgs::Image& ros_image) const
{
  ros_image.header = header;
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = encoding;
  ros_image.is_bigendian = false;
  ros_image.step = image.step;
  size_t size = image.step * image.rows;
  ros_image.data.resize(size);
  memcpy((char*)(&ros_image.data[0]), image.data, size);
}

// Deep copy data, returnee is mutable
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopy(*source);
}

CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                    const std::string& encoding)
{
  // Copy metadata
  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = source.header;

  // Construct matrix pointing to source data
  int source_type = getCvType(source.encoding);
  const cv::Mat tmp((int)source.height, (int)source.width, source_type,
                    const_cast<uint8_t*>(&source.data[0]), (size_t)source.step);
  // Copy to new buffer if same encoding requested
  if (encoding.empty() || encoding == source.encoding)
  {
    ptr->encoding = source.encoding;
    tmp.copyTo(ptr->image);
  }
  else
  {
    // Convert the source data to the desired encoding
    int destination_type = getCvType(encoding);
    Format source_format = getFormat(source.encoding, source_type);
    Format destination_format = getFormat(encoding, destination_type);
    if (source_format == INVALID || destination_format == INVALID)
      throw Exception("Unsupported conversion from [" + source.encoding +
                      "] to [" + encoding + "]");

    int conversion_code = getConversionCode(source_format, destination_format);
    if (conversion_code == SAME_FORMAT)
    {
      // Same number of channels, but different bit depth
      /// @todo Should do scaling, e.g. for MONO16 -> MONO8
      tmp.convertTo(ptr->image, destination_type);
    }
    else
    {
      // Perform color conversion
      cv::cvtColor(tmp, ptr->image, conversion_code);
    }
    ptr->encoding = encoding;
  }

  return ptr;
}

// Share const data, returnee is immutable
CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                          const std::string& encoding)
{
  return toCvShare(*source, source, encoding);
}

CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                          const boost::shared_ptr<void const>& tracked_object,
                          const std::string& encoding)
{
  if (!encoding.empty() && source.encoding != encoding)
    return toCvCopy(source, encoding);

  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = source.header;
  ptr->encoding = source.encoding;
  ptr->tracked_object_ = tracked_object;
  int type = getCvType(source.encoding);
  ptr->image = cv::Mat(source.height, source.width, type,
                       const_cast<uchar*>(&source.data[0]), source.step);
  return ptr;
}

} //namespace cv_bridge
