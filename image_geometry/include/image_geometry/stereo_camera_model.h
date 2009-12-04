#ifndef IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H

#include "image_geometry/pinhole_camera_model.h"

namespace image_geometry {

class StereoCameraModel
{
public:
  StereoCameraModel();

  StereoCameraModel(const StereoCameraModel& other);

  // Update in image/info callback
  void fromCameraInfo(const sensor_msgs::CameraInfo& left,
                      const sensor_msgs::CameraInfo& right);

  // Read-only access to both mono camera models
  const PinholeCameraModel& left() const;
  const PinholeCameraModel& right() const;

  // Projection functions, 3d points are in tfFrame()
  std::string tfFrame() const; // Should be same for both cameras

  void projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const;

  /// @todo Function for projecting disparity image to point cloud

  // Arguments for OpenCV functions
  const cv::Mat_<double>& reprojectionMatrix() const; // Q, 4x4

  // Accessors for specific parameters
  double baseline() const; //Tx

  /// @todo Maybe more convenience functions from CvStereoCamModel in stereo_util, like getDelta_.

private:
  bool initialized_;
  PinholeCameraModel left_, right_;
  cv::Mat_<double> Q_;

  void updateQ();
};


/* Trivial inline functions */
inline const PinholeCameraModel& StereoCameraModel::left() const  { return left_; }
inline const PinholeCameraModel& StereoCameraModel::right() const { return right_; }

inline std::string StereoCameraModel::tfFrame() const { return left_.tfFrame(); }

inline const cv::Mat_<double>& StereoCameraModel::reprojectionMatrix() const { return Q_; }

inline double StereoCameraModel::baseline() const
{
  /// @todo Currently assuming horizontal baseline
  return -right_.projectionMatrix()(0,3) / right_.fx();
}

} //namespace image_geometry

#endif
