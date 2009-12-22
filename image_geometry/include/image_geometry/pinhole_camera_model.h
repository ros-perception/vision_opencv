#ifndef IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H

#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>

namespace image_geometry {

/**
 * \brief Simplifies interpreting images geometrically using the parameters from
 * sensor_msgs/CameraInfo.
 */
class PinholeCameraModel
{
public:
  
  PinholeCameraModel();

  PinholeCameraModel(const PinholeCameraModel& other);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
   */
  void fromCameraInfo(const sensor_msgs::CameraInfo& msg);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
   */
  void fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

  /**
   * \brief Get the name of the camera coordinate frame in tf.
   */
  std::string tfFrame() const;

  /**
   * \brief Project a 3d point to rectified pixel coordinates.
   *
   * This is the inverse of projectPixelTo3dRay().
   *
   * \param xyz 3d point in the camera coordinate frame
   * \param[out] uv_rect Rectified pixel coordinates
   */
  void project3dToPixel(const cv::Point3d& xyz, cv::Point2d& uv_rect) const;

  /**
   * \brief Project a rectified pixel to a 3d ray.
   *
   * Returns the unit vector in the camera coordinate frame in the direction of rectified
   * pixel (u,v) in the image plane. This is the inverse of project3dToPixel().
   *
   * \param uv_rect Rectified pixel coordinates
   * \param[out] ray 3d ray passing through (u,v).
   */
  void projectPixelTo3dRay(const cv::Point2d& uv_rect, cv::Point3d& ray) const;

  /**
   * \brief Rectify a raw camera image.
   */
  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  /**
   * \brief Apply camera distortion to a rectified image.
   */
  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw) const;

  /**
   * \brief Compute the rectified image coordinates of a pixel in the raw image.
   */
  void rectifyPoint(const cv::Point2d& uv_raw, cv::Point2d& uv_rect) const;

  /**
   * \brief Compute the raw image coordinates of a pixel in the rectified image.
   */
  void unrectifyPoint(const cv::Point2d& uv_rect, cv::Point2d& uv_raw) const;

  /**
   * \brief Returns the original camera matrix.
   */
  const cv::Mat_<double>& intrinsicMatrix() const;

  /**
   * \brief Returns the distortion coefficients.
   */
  const cv::Mat_<double>& distortionCoeffs() const;

  /**
   * \brief Returns the rotation matrix.
   */
  const cv::Mat_<double>& rotationMatrix() const;

  /**
   * \brief Returns the projection matrix.
   */
  const cv::Mat_<double>& projectionMatrix() const;

  /**
   * \brief Returns the focal length (pixels) in x direction of the rectified image.
   */
  double fx() const;

  /**
   * \brief Returns the focal length (pixels) in y direction of the rectified image.
   */
  double fy() const;

  /**
   * \brief Returns the x coordinate of the optical center.
   */
  double cx() const;

  /**
   * \brief Returns the y coordinate of the optical center.
   */
  double cy() const;

  /**
   * \brief Returns the image height.
   */
  uint32_t height() const;

  /**
   * \brief Returns the image width.
   */
  uint32_t width() const;

private:
  bool initialized_;
  bool has_distortion_, has_roi_;
  
  sensor_msgs::CameraInfo cam_info_;
  cv::Mat_<double> K_, D_, R_, P_;

  mutable bool parameters_changed_, roi_changed_;
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
