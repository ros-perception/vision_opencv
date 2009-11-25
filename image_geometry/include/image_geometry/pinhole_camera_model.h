#ifndef IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H

#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>

namespace image_geometry {

typedef CvPoint2D64f Point2d;
typedef CvPoint3D64f Point3d;

class PinholeCameraModel
{
public:
  PinholeCameraModel();

  PinholeCameraModel(const PinholeCameraModel& other);

  // Update in image/info callback
  void fromCameraInfo(const sensor_msgs::CameraInfo& msg);
  void fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

  // Projection functions, 3d points are in tfFrame()
  std::string tfFrame() const;
  
  void project3dToPixel(const Point3d& xyz, Point2d& uv_rect) const;

  void projectPixelTo3dRay(const Point2d& uv_rect, Point3d& ray) const;

  // Rectification
  void rectifyImage(const CvArr* raw, CvArr* rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  void unrectifyImage(const CvArr* rectified, CvArr* raw) const;

  void rectifyPoint(const Point2d& uv_raw, Point2d& uv_rect) const;

  void unrectifyPoint(const Point2d& uv_rect, Point2d& uv_raw) const;

  // Arguments for OpenCV functions
  const CvMat* intrinsicMatrix() const;  // K, also called camera_matrix in cv docs
  const CvMat* distortionCoeffs() const; // D, 1x4 or 1x5?
  const CvMat* rotationMatrix() const;   // R, rectificationMatrix?
  const CvMat* projectionMatrix() const; // P, 3x4

  // Accessors for specific parameters
  double fx() const;
  double fy() const;
  double cx() const;
  double cy() const;
  uint32_t height() const;
  uint32_t width() const;

private:
  bool initialized_;
  bool parameters_changed_, roi_changed_;
  bool has_distortion_, has_roi_;
  
  sensor_msgs::CameraInfo cam_info_;
  CvMat K_, D_, R_, P_;
  
  mutable cv::WImageBuffer1_f undistort_map_x_, undistort_map_y_;
  mutable cv::WImageBuffer1_f roi_undistort_map_x_, roi_undistort_map_y_;

  void initUndistortMaps() const;
};


/* Trivial inline functions */
inline std::string PinholeCameraModel::tfFrame() const
{
  assert(initialized_);
  return cam_info_.header.frame_id;
}

/// @todo assert initialized in all these
inline const CvMat* PinholeCameraModel::intrinsicMatrix() const  { return &K_; }
inline const CvMat* PinholeCameraModel::distortionCoeffs() const { return &D_; }
inline const CvMat* PinholeCameraModel::rotationMatrix() const   { return &R_; }
inline const CvMat* PinholeCameraModel::projectionMatrix() const { return &P_; }

inline double PinholeCameraModel::fx() const { return cam_info_.P[0]; }
inline double PinholeCameraModel::fy() const { return cam_info_.P[5]; }
inline double PinholeCameraModel::cx() const { return cam_info_.P[2]; }
inline double PinholeCameraModel::cy() const { return cam_info_.P[6]; }
inline uint32_t PinholeCameraModel::height() const { return cam_info_.height; }
inline uint32_t PinholeCameraModel::width() const  { return cam_info_.width; }

} //namespace image_geometry

#endif
