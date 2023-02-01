// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CV_BRIDGE__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_
#define CV_BRIDGE__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_

#include <cstddef>
#include <memory>
#include <variant>

#include "opencv2/core/mat.hpp"

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/visibility_control.h"

#include <optional>

namespace cv_bridge
{
namespace detail
{
// TODO(audrow): Replace with std::endian when C++ 20 is available
// https://en.cppreference.com/w/cpp/types/endian
enum class endian
{
#ifdef _WIN32
  little = 0,
  big    = 1,
  native = little
#else
  little = __ORDER_LITTLE_ENDIAN__,
  big    = __ORDER_BIG_ENDIAN__,
  native = __BYTE_ORDER__
#endif
};

}  // namespace detail


/// A potentially owning, potentially non-owning, container of a cv::Mat and ROS header.
/**
 * The two main use cases for this are publishing user controlled data, and
 * recieving data from the middleware that may have been a ROS message
 * originally or may have been an cv::Mat originally.
 *
 * In the first case, publishing user owned data, the user will want to provide
 * their own cv::Mat.
 * The cv::Mat may own the data or it may not, so in the latter case, it is up
 * to the user to ensure the data the cv::Mat points to remains valid as long
 * as the middleware needs it.
 *
 * In the second case, receiving data from the middleware, the middleware will
 * either give a new ROSCvMatContainer which owns a sensor_msgs::msg::Image or
 * it will give a ROSCvMatContainer that was previously published by the user
 * (in the case of intra-process communication).
 * If the container owns the sensor_msgs::msg::Image, then the cv::Mat will just
 * reference data field of this message, so the container needs to be kept.
 * If the container was published by the user it may or may not own the data
 * and the cv::Mat it contains may or may not own the data.
 *
 * For these reasons, it is advisable to use cv::Mat::clone() if you intend to
 * copy the cv::Mat and let this container go.
 *
 * For more details about the ownership behavior of cv::Mat see documentation
 * for these methods of cv::Mat:
 *
 *   - template<typename _Tp > cv::Mat::Mat(const std::vector<_Tp> &, bool)
 *   - Mat & cv::Mat::operator=(const Mat &)
 *   - void cv::Mat::addref()
 *   - void cv::Mat::release()
 *
 */
class ROSCvMatContainer
{
  static constexpr bool is_bigendian_system = detail::endian::native == detail::endian::big;

public:
  using SensorMsgsImageStorageType = std::variant<
    std::nullptr_t,
    std::unique_ptr<sensor_msgs::msg::Image>,
    std::shared_ptr<sensor_msgs::msg::Image>
  >;

  CV_BRIDGE_PUBLIC
  ROSCvMatContainer() = default;

  CV_BRIDGE_PUBLIC
  explicit ROSCvMatContainer(const ROSCvMatContainer & other)
  : header_(other.header_), frame_(other.frame_.clone()), is_bigendian_(other.is_bigendian_)
  {
    if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
      storage_ = std::get<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_);
    } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
      storage_ = std::make_unique<sensor_msgs::msg::Image>(
        *std::get<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_));
    }
  }

  CV_BRIDGE_PUBLIC
  ROSCvMatContainer & operator=(const ROSCvMatContainer & other)
  {
    if (this != &other) {
      header_ = other.header_;
      frame_ = other.frame_.clone();
      is_bigendian_ = other.is_bigendian_;
      if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
        storage_ = std::get<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_);
      } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
        storage_ = std::make_unique<sensor_msgs::msg::Image>(
          *std::get<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_));
      } else if (std::holds_alternative<std::nullptr_t>(other.storage_)) {
        storage_ = nullptr;
      }
    }
    return *this;
  }

  /// Store an owning pointer to a sensor_msg::msg::Image, and create a cv::Mat that references it.
  CV_BRIDGE_PUBLIC
  explicit ROSCvMatContainer(std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image);

  /// Store an owning pointer to a sensor_msg::msg::Image, and create a cv::Mat that references it.
  CV_BRIDGE_PUBLIC
  explicit ROSCvMatContainer(std::shared_ptr<sensor_msgs::msg::Image> shared_sensor_msgs_image);

  /// Shallow copy the given cv::Mat into this class, but do not own the data directly.
  CV_BRIDGE_PUBLIC
  ROSCvMatContainer(
    const cv::Mat & mat_frame,
    const std_msgs::msg::Header & header,
    bool is_bigendian = is_bigendian_system,
    std::optional<std::string> encoding_override = std::nullopt);

  /// Move the given cv::Mat into this class.
  CV_BRIDGE_PUBLIC
  ROSCvMatContainer(
    cv::Mat && mat_frame,
    const std_msgs::msg::Header & header,
    bool is_bigendian = is_bigendian_system,
    std::optional<std::string> encoding_override = std::nullopt);

  /// Copy the sensor_msgs::msg::Image into this contain and create a cv::Mat that references it.
  CV_BRIDGE_PUBLIC
  explicit ROSCvMatContainer(const sensor_msgs::msg::Image & sensor_msgs_image);

  /// Return true if this class owns the data the cv_mat references.
  /**
   * Note that this does not check if the cv::Mat owns its own data, only if
   * this class owns a sensor_msgs::msg::Image that the cv::Mat references.
   */
  CV_BRIDGE_PUBLIC
  bool
  is_owning() const;

  /// Const access the cv::Mat in this class.
  CV_BRIDGE_PUBLIC
  const cv::Mat &
  cv_mat() const;

  /// Get a shallow copy of the cv::Mat that is in this class.
  /**
   * Note that if you want to let this container go out of scope you should
   * make a deep copy with cv::Mat::clone() beforehand.
   */
  CV_BRIDGE_PUBLIC
  cv::Mat
  cv_mat();

  /// Const access the ROS Header.
  CV_BRIDGE_PUBLIC
  const std_msgs::msg::Header &
  header() const;

  /// Access the ROS Header.
  CV_BRIDGE_PUBLIC
  std_msgs::msg::Header &
  header();

  /// Get shared const pointer to the sensor_msgs::msg::Image if available, otherwise nullptr.
  CV_BRIDGE_PUBLIC
  std::shared_ptr<const sensor_msgs::msg::Image>
  get_sensor_msgs_msg_image_pointer() const;

  /// Get copy as a unique pointer to the sensor_msgs::msg::Image.
  CV_BRIDGE_PUBLIC
  std::unique_ptr<sensor_msgs::msg::Image>
  get_sensor_msgs_msg_image_pointer_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::Image.
  CV_BRIDGE_PUBLIC
  sensor_msgs::msg::Image
  get_sensor_msgs_msg_image_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::Image.
  CV_BRIDGE_PUBLIC
  void
  get_sensor_msgs_msg_image_copy(sensor_msgs::msg::Image & sensor_msgs_image) const;

  /// Return true if the data is stored in big endian, otherwise return false.
  CV_BRIDGE_PUBLIC
  bool
  is_bigendian() const;
  
  /// Return the encoding override if provided.
  CV_BRIDGE_PUBLIC
  std::optional<std::string>
  encoding_override() const;

private:
  std_msgs::msg::Header header_;
  cv::Mat frame_;
  SensorMsgsImageStorageType storage_;
  bool is_bigendian_;
  std::optional<std::string> encoding_override_;
};

}  // namespace cv_bridge

template<>
struct rclcpp::TypeAdapter<cv_bridge::ROSCvMatContainer, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cv_bridge::ROSCvMatContainer;
  using ros_message_type = sensor_msgs::msg::Image;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.height = source.cv_mat().rows;
    destination.width = source.cv_mat().cols;
    const auto& encoding_override = source.encoding_override();
    if (encoding_override.has_value() && !encoding_override.value().empty())
    {
      destination.encoding = encoding_override.value();
    }
    else
    {
      switch (source.cv_mat().type()) {
      case CV_8UC1:
        destination.encoding = "mono8";
        break;
      case CV_8UC3:
        destination.encoding = "bgr8";
        break;
      case CV_16SC1:
        destination.encoding = "mono16";
        break;
      case CV_8UC4:
        destination.encoding = "rgba8";
        break;
      default:
        throw std::runtime_error("unsupported encoding type");
      }    
    }
    destination.step = static_cast<sensor_msgs::msg::Image::_step_type>(source.cv_mat().step);
    size_t size = source.cv_mat().step * source.cv_mat().rows;
    destination.data.resize(size);
    memcpy(&destination.data[0], source.cv_mat().data, size);
    destination.header = source.header();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = cv_bridge::ROSCvMatContainer(source);
  }
};

#endif  // CV_BRIDGE__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_
