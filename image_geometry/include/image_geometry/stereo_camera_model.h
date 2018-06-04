#ifndef IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H

#include "image_geometry/pinhole_camera_model.h"
#include "image_geometry/visibility_control.hpp"

namespace image_geometry {

/**
 * \brief Simplifies interpreting stereo image pairs geometrically using the
 * parameters from the left and right sensor_msgs/CameraInfo.
 */
class StereoCameraModel
{
public:
  IMAGE_GEOMETRY_PUBLIC
  StereoCameraModel();

  IMAGE_GEOMETRY_PUBLIC
  StereoCameraModel(const StereoCameraModel& other);

  IMAGE_GEOMETRY_PUBLIC
  StereoCameraModel& operator=(const StereoCameraModel& other);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
   */
  IMAGE_GEOMETRY_PUBLIC
  bool fromCameraInfo(const sensor_msgs::msg::CameraInfo& left,
                      const sensor_msgs::msg::CameraInfo& right);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
   */
  IMAGE_GEOMETRY_PUBLIC
  bool fromCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right);

  /**
   * \brief Get the left monocular camera model.
   */
  IMAGE_GEOMETRY_PUBLIC
  const PinholeCameraModel& left() const;

  /**
   * \brief Get the right monocular camera model.
   */
  IMAGE_GEOMETRY_PUBLIC
  const PinholeCameraModel& right() const;

  /**
   * \brief Get the name of the camera coordinate frame in tf.
   *
   * For stereo cameras, both the left and right CameraInfo should be in the left
   * optical frame.
   */
  IMAGE_GEOMETRY_PUBLIC
  std::string tfFrame() const;

  /**
   * \brief Project a rectified pixel with disparity to a 3d point.
   */
  IMAGE_GEOMETRY_PUBLIC
  void projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const;

  /**
   * \brief Project a disparity image to a 3d point cloud.
   *
   * If handleMissingValues = true, all points with minimal disparity (outliers) have
   * Z set to MISSING_Z (currently 10000.0).
   */
  IMAGE_GEOMETRY_PUBLIC
  void projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& point_cloud,
                                 bool handleMissingValues = false) const;
  static const double MISSING_Z;
  
  /**
   * \brief Returns the disparity reprojection matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx44d& reprojectionMatrix() const;

  /**
   * \brief Returns the horizontal baseline in world coordinates.
   */
  IMAGE_GEOMETRY_PUBLIC
  double baseline() const;

  /**
   * \brief Returns the depth at which a point is observed with a given disparity.
   *
   * This is the inverse of getDisparity().
   */
  IMAGE_GEOMETRY_PUBLIC
  double getZ(double disparity) const;

  /**
   * \brief Returns the disparity observed for a point at depth Z.
   *
   * This is the inverse of getZ().
   */
  IMAGE_GEOMETRY_PUBLIC
  double getDisparity(double Z) const;

  /**
   * \brief Returns true if the camera has been initialized
   */
  IMAGE_GEOMETRY_PUBLIC
  bool initialized() const { return left_.initialized() && right_.initialized(); }
protected:
  PinholeCameraModel left_, right_;
  cv::Matx44d Q_;

  IMAGE_GEOMETRY_PUBLIC
  void updateQ();
};


/* Trivial inline functions */
IMAGE_GEOMETRY_PUBLIC
inline const PinholeCameraModel& StereoCameraModel::left() const  { return left_; }
IMAGE_GEOMETRY_PUBLIC
inline const PinholeCameraModel& StereoCameraModel::right() const { return right_; }

IMAGE_GEOMETRY_PUBLIC
inline std::string StereoCameraModel::tfFrame() const { return left_.tfFrame(); }

IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx44d& StereoCameraModel::reprojectionMatrix() const { return Q_; }

IMAGE_GEOMETRY_PUBLIC
inline double StereoCameraModel::baseline() const
{
  /// @todo Currently assuming horizontal baseline
  return -right_.Tx() / right_.fx();
}

IMAGE_GEOMETRY_PUBLIC
inline double StereoCameraModel::getZ(double disparity) const
{
  assert( initialized() );
  return -right_.Tx() / (disparity - (left().cx() - right().cx()));
}

IMAGE_GEOMETRY_PUBLIC
inline double StereoCameraModel::getDisparity(double Z) const
{
  assert( initialized() );
  return -right_.Tx() / Z + (left().cx() - right().cx()); ;
}

} //namespace image_geometry

#endif
