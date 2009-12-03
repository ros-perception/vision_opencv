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

  void project3dToPixel(const Point3d& xyz, Point2d& left_uv_rect) const;
  void project3dToPixel(const Point3d& xyz, Point2d& left_uv_rect, Point2d& right_uv_rect) const;

  void projectPixelTo3d(const Point2d& left_uv_rect, float disparity, Point3d& xyz) const;

  /// @todo Function for projecting disparity image to point cloud

  // Arguments for OpenCV functions
  const CvMat* reprojectionMatrix() const; // Q, 4x4

  // Accessors for specific parameters
  double baseline() const; //Tx

  /// @todo Maybe more convenience functions from CvStereoCamModel in stereo_util

private:
  PinholeCameraModel left_, right_;
  double Q_buf_[16];
  CvMat Q_;
  bool initialized_;
};


/* Trivial inline functions */
inline const StereoCameraModel::PinholeCameraModel& left() const  { return left_; }
inline const StereoCameraModel::PinholeCameraModel& right() const { return right_; }

inline std::string StereoCameraModel::tfFrame() const { return left_.tfFrame(); }

inline const CvMat* StereoCameraModel::reprojectionMatrix() const { return &Q_; }

inline double StereoCameraModel::baseline() const
{
  /// @todo Assuming horizontal baseline ok?
  return -right_.projectionMatrix()->data.db[3] / right_.fx();
}

} //namespace image_geometry

#endif
