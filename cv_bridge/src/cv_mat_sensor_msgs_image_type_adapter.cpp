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

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "opencv2/core/mat.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp"

namespace cv_bridge
{

namespace
{
int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else if (encoding == "yuv422") {
    return CV_8UC2;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

/// Create a cv::Mat that references the data of image, after checking that the data is as
/// large as the geometry of the image claims.
/**
 * The image may come straight from the network, so nothing in it can be trusted: without these
 * checks a message with a short data field makes every user of the cv::Mat read out of bounds.
 * The products are computed in 64 bits because the fields of the message are 32 bits wide.
 */
cv::Mat
mat_referencing_image(sensor_msgs::msg::Image & image)
{
  const int type = encoding2mat_type(image.encoding);
  const uint64_t max_dimension = static_cast<uint64_t>(std::numeric_limits<int>::max());
  if (image.height > max_dimension || image.width > max_dimension) {
    throw std::runtime_error("Image is wrongly formed: height or width is too large");
  }
  const uint64_t min_step = static_cast<uint64_t>(image.width) * CV_ELEM_SIZE(type);
  if (image.step < min_step) {
    throw std::runtime_error("Image is wrongly formed: step < width * pixel size");
  }
  if (static_cast<uint64_t>(image.height) * image.step != image.data.size()) {
    throw std::runtime_error("Image is wrongly formed: height * step != size");
  }
  return cv::Mat(
    static_cast<int>(image.height), static_cast<int>(image.width), type,
    image.data.data(), image.step);
}

template<typename T>
struct NotNull
{
  NotNull(const T * pointer_in, const char * msg)
  : pointer(pointer_in)
  {
    if (pointer == nullptr) {
      throw std::invalid_argument(msg);
    }
  }

  const T * pointer;
};

}  // namespace

ROSCvMatContainer::ROSCvMatContainer(
  std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image)
: header_(NotNull(
      unique_sensor_msgs_image.get(),
      "unique_sensor_msgs_image cannot be nullptr"
).pointer->header),
  frame_(mat_referencing_image(*unique_sensor_msgs_image)),
  storage_(std::move(unique_sensor_msgs_image))
{}

ROSCvMatContainer::ROSCvMatContainer(
  std::shared_ptr<sensor_msgs::msg::Image> shared_sensor_msgs_image)
: header_(NotNull(
      shared_sensor_msgs_image.get(),
      "shared_sensor_msgs_image cannot be nullptr"
).pointer->header),
  frame_(mat_referencing_image(*shared_sensor_msgs_image)),
  storage_(shared_sensor_msgs_image)
{}

ROSCvMatContainer::ROSCvMatContainer(
  const cv::Mat & mat_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian,
  std::optional<std::string> encoding_override)
: header_(header),
  frame_(mat_frame),
  storage_(nullptr),
  is_bigendian_(is_bigendian),
  encoding_override_(encoding_override)
{}

ROSCvMatContainer::ROSCvMatContainer(
  cv::Mat && mat_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian,
  std::optional<std::string> encoding_override)
: header_(header),
  frame_(std::forward<cv::Mat>(mat_frame)),
  storage_(nullptr),
  is_bigendian_(is_bigendian),
  encoding_override_(encoding_override)
{}

ROSCvMatContainer::ROSCvMatContainer(
  const sensor_msgs::msg::Image & sensor_msgs_image)
: ROSCvMatContainer(std::make_unique<sensor_msgs::msg::Image>(sensor_msgs_image))
{}

bool
ROSCvMatContainer::is_owning() const
{
  return std::holds_alternative<std::nullptr_t>(storage_);
}

const cv::Mat &
ROSCvMatContainer::cv_mat() const
{
  return frame_;
}

cv::Mat
ROSCvMatContainer::cv_mat()
{
  return frame_;
}

const std_msgs::msg::Header &
ROSCvMatContainer::header() const
{
  return header_;
}

std_msgs::msg::Header &
ROSCvMatContainer::header()
{
  return header_;
}

std::shared_ptr<const sensor_msgs::msg::Image>
ROSCvMatContainer::get_sensor_msgs_msg_image_pointer() const
{
  if (!std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(storage_)) {
    return nullptr;
  }
  return std::get<std::shared_ptr<sensor_msgs::msg::Image>>(storage_);
}

std::unique_ptr<sensor_msgs::msg::Image>
ROSCvMatContainer::get_sensor_msgs_msg_image_pointer_copy() const
{
  auto unique_image = std::make_unique<sensor_msgs::msg::Image>();
  this->get_sensor_msgs_msg_image_copy(*unique_image);
  return unique_image;
}

void ROSCvMatContainer::get_sensor_msgs_msg_image_copy(
  sensor_msgs::msg::Image& sensor_msgs_image) const
{
  fill_sensor_msgs_image_from_cvmat(frame_, sensor_msgs_image, header_, encoding_override_);
}

bool
ROSCvMatContainer::is_bigendian() const
{
  return is_bigendian_;
}

std::optional<std::string>
ROSCvMatContainer::encoding_override() const
{
  return encoding_override_;
}

}  // namespace cv_bridge
