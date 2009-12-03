#ifndef IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H

#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>

namespace image_geometry {

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
  
  void project3dToPixel(const cv::Point3d& xyz, cv::Point2d& uv_rect) const;

  void projectPixelTo3dRay(const cv::Point2d& uv_rect, cv::Point3d& ray) const;

  // Rectification
  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw) const;

  void rectifyPoint(const cv::Point2d& uv_raw, cv::Point2d& uv_rect) const;

  void unrectifyPoint(const cv::Point2d& uv_rect, cv::Point2d& uv_raw) const;

  // Arguments for OpenCV functions
  const cv::Mat_<double>& intrinsicMatrix() const;  // K, also called camera_matrix in cv docs
  const cv::Mat_<double>& distortionCoeffs() const; // D, 1x4 or 1x5?
  const cv::Mat_<double>& rotationMatrix() const;   // R, rectificationMatrix?
  const cv::Mat_<double>& projectionMatrix() const; // P, 3x4

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
  cv::Mat_<double> K_, D_, R_, P_;

  mutable cv::Mat undistort_map_x_, undistort_map_y_;
  mutable cv::Mat roi_undistort_map_x_, roi_undistort_map_y_;

  void initUndistortMaps() const;
};


/* Trivial inline functions */
inline std::string PinholeCameraModel::tfFrame() const
{
  assert(initialized_);
  return cam_info_.header.frame_id;
}

/// @todo assert initialized in all these
inline const cv::Mat_<double>& PinholeCameraModel::intrinsicMatrix() const  { return K_; }
inline const cv::Mat_<double>& PinholeCameraModel::distortionCoeffs() const { return D_; }
inline const cv::Mat_<double>& PinholeCameraModel::rotationMatrix() const   { return R_; }
inline const cv::Mat_<double>& PinholeCameraModel::projectionMatrix() const { return P_; }

inline double PinholeCameraModel::fx() const { return P_(0,0); }
inline double PinholeCameraModel::fy() const { return P_(1,1); }
inline double PinholeCameraModel::cx() const { return P_(0,2); }
inline double PinholeCameraModel::cy() const { return P_(1,2); }
inline uint32_t PinholeCameraModel::height() const { return cam_info_.height; }
inline uint32_t PinholeCameraModel::width() const  { return cam_info_.width; }

} //namespace image_geometry

#endif
