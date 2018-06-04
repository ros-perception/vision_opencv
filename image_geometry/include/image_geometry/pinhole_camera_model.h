#ifndef IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H

#include "image_geometry/visibility_control.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <stdexcept>
#include <string>

namespace image_geometry {

class Exception : public std::runtime_error
{
public:
  Exception(const std::string& description) : std::runtime_error(description) {}
};

/**
 * \brief Simplifies interpreting images geometrically using the parameters from
 * sensor_msgs/CameraInfo.
 */
class PinholeCameraModel
{
public:
  IMAGE_GEOMETRY_PUBLIC
  PinholeCameraModel();

  IMAGE_GEOMETRY_PUBLIC
  PinholeCameraModel(const PinholeCameraModel& other);

  IMAGE_GEOMETRY_PUBLIC
  PinholeCameraModel& operator=(const PinholeCameraModel& other);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
   */
  IMAGE_GEOMETRY_PUBLIC
  bool fromCameraInfo(const sensor_msgs::msg::CameraInfo& msg);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
   */
  IMAGE_GEOMETRY_PUBLIC
  bool fromCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  /**
   * \brief Get the name of the camera coordinate frame in tf.
   */
  IMAGE_GEOMETRY_PUBLIC
  std::string tfFrame() const;

  /**
   * \brief Get the time stamp associated with this camera model.
   */
  IMAGE_GEOMETRY_PUBLIC
  builtin_interfaces::msg::Time stamp() const;

  /**
   * \brief The resolution at which the camera was calibrated.
   *
   * The maximum resolution at which the camera can be used with the current
   * calibration; normally this is the same as the imager resolution.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Size fullResolution() const;

  /**
   * \brief The resolution of the current rectified image.
   *
   * The size of the rectified image associated with the latest CameraInfo, as
   * reduced by binning/ROI and affected by distortion. If binning and ROI are
   * not in use, this is the same as fullResolution().
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Size reducedResolution() const;

  IMAGE_GEOMETRY_PUBLIC
  cv::Point2d toFullResolution(const cv::Point2d& uv_reduced) const;

  IMAGE_GEOMETRY_PUBLIC
  cv::Rect toFullResolution(const cv::Rect& roi_reduced) const;

  IMAGE_GEOMETRY_PUBLIC
  cv::Point2d toReducedResolution(const cv::Point2d& uv_full) const;

  IMAGE_GEOMETRY_PUBLIC
  cv::Rect toReducedResolution(const cv::Rect& roi_full) const;

  /**
   * \brief The current raw ROI, as used for capture by the camera driver.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Rect rawRoi() const;

  /**
   * \brief The current rectified ROI, which best fits the raw ROI.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Rect rectifiedRoi() const;

  /**
   * \brief Project a 3d point to rectified pixel coordinates.
   *
   * This is the inverse of projectPixelTo3dRay().
   *
   * \param xyz 3d point in the camera coordinate frame
   * \return (u,v) in rectified pixel coordinates
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Point2d project3dToPixel(const cv::Point3d& xyz) const;

  /**
   * \brief Project a rectified pixel to a 3d ray.
   *
   * Returns the unit vector in the camera coordinate frame in the direction of rectified
   * pixel (u,v) in the image plane. This is the inverse of project3dToPixel().
   *
   * In 1.4.x, the vector has z = 1.0. Previously, this function returned a unit vector.
   *
   * \param uv_rect Rectified pixel coordinates
   * \return 3d ray passing through (u,v)
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const;

  /**
   * \brief Rectify a raw camera image.
   */
  IMAGE_GEOMETRY_PUBLIC
  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = cv::INTER_LINEAR) const;

  /**
   * \brief Apply camera distortion to a rectified image.
   */
  IMAGE_GEOMETRY_PUBLIC
  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw,
                      int interpolation = cv::INTER_LINEAR) const;

  /**
   * \brief Compute the rectified image coordinates of a pixel in the raw image.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Point2d rectifyPoint(const cv::Point2d& uv_raw) const;

  /**
   * \brief Compute the raw image coordinates of a pixel in the rectified image.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Point2d unrectifyPoint(const cv::Point2d& uv_rect) const;

  /**
   * \brief Compute the rectified ROI best fitting a raw ROI.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Rect rectifyRoi(const cv::Rect& roi_raw) const;

  /**
   * \brief Compute the raw ROI best fitting a rectified ROI.
   */
  IMAGE_GEOMETRY_PUBLIC
  cv::Rect unrectifyRoi(const cv::Rect& roi_rect) const;

  /**
   * \brief Returns the camera info message held internally
   */
  IMAGE_GEOMETRY_PUBLIC
  const sensor_msgs::msg::CameraInfo& cameraInfo() const;

  /**
   * \brief Returns the original camera matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx33d& intrinsicMatrix() const;

  /**
   * \brief Returns the distortion coefficients.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Mat_<double>& distortionCoeffs() const;

  /**
   * \brief Returns the rotation matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx33d& rotationMatrix() const;

  /**
   * \brief Returns the projection matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx34d& projectionMatrix() const;

  /**
   * \brief Returns the original camera matrix for full resolution.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx33d& fullIntrinsicMatrix() const;

  /**
   * \brief Returns the projection matrix for full resolution.
   */
  IMAGE_GEOMETRY_PUBLIC
  const cv::Matx34d& fullProjectionMatrix() const;

  /**
   * \brief Returns the focal length (pixels) in x direction of the rectified image.
   */
  IMAGE_GEOMETRY_PUBLIC
  double fx() const;

  /**
   * \brief Returns the focal length (pixels) in y direction of the rectified image.
   */
  IMAGE_GEOMETRY_PUBLIC
  double fy() const;

  /**
   * \brief Returns the x coordinate of the optical center.
   */
  IMAGE_GEOMETRY_PUBLIC
  double cx() const;

  /**
   * \brief Returns the y coordinate of the optical center.
   */
  IMAGE_GEOMETRY_PUBLIC
  double cy() const;

  /**
   * \brief Returns the x-translation term of the projection matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  double Tx() const;

  /**
   * \brief Returns the y-translation term of the projection matrix.
   */
  IMAGE_GEOMETRY_PUBLIC
  double Ty() const;

  /**
   * \brief Returns the number of columns in each bin.
   */
  IMAGE_GEOMETRY_PUBLIC
  uint32_t binningX() const;

  /**
   * \brief Returns the number of rows in each bin.
   */
  IMAGE_GEOMETRY_PUBLIC
  uint32_t binningY() const;
  
  /**
   * \brief Compute delta u, given Z and delta X in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaX().
   *
   * \param deltaX Delta X, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  IMAGE_GEOMETRY_PUBLIC
  double getDeltaU(double deltaX, double Z) const;

  /**
   * \brief Compute delta v, given Z and delta Y in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaY().
   *
   * \param deltaY Delta Y, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  IMAGE_GEOMETRY_PUBLIC
  double getDeltaV(double deltaY, double Z) const;

  /**
   * \brief Compute delta X, given Z in Cartesian space and delta u in pixels.
   *
   * For given Z, this is the inverse of getDeltaU().
   *
   * \param deltaU Delta u, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  IMAGE_GEOMETRY_PUBLIC
  double getDeltaX(double deltaU, double Z) const;

  /**
   * \brief Compute delta Y, given Z in Cartesian space and delta v in pixels.
   *
   * For given Z, this is the inverse of getDeltaV().
   *
   * \param deltaV Delta v, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  IMAGE_GEOMETRY_PUBLIC
  double getDeltaY(double deltaV, double Z) const;

  /**
   * \brief Returns true if the camera has been initialized
   */
  IMAGE_GEOMETRY_PUBLIC
  bool initialized() const { return (bool)cache_; }

protected:
  sensor_msgs::msg::CameraInfo cam_info_;
  cv::Mat_<double> D_;           // Unaffected by binning, ROI
  cv::Matx33d R_;           // Unaffected by binning, ROI
  cv::Matx33d K_;           // Describe current image (includes binning, ROI)
  cv::Matx34d P_;           // Describe current image (includes binning, ROI)
  cv::Matx33d K_full_; // Describe full-res image, needed for full maps
  cv::Matx34d P_full_; // Describe full-res image, needed for full maps

  // Use PIMPL here so we can change internals in patch updates if needed
  struct Cache;
  std::shared_ptr<Cache> cache_; // Holds cached data for internal use

  IMAGE_GEOMETRY_PUBLIC
  void initRectificationMaps() const;

  friend class StereoCameraModel;
};


/* Trivial inline functions */
IMAGE_GEOMETRY_PUBLIC
inline std::string PinholeCameraModel::tfFrame() const
{
  assert( initialized() );
  return cam_info_.header.frame_id;
}

IMAGE_GEOMETRY_PUBLIC
inline builtin_interfaces::msg::Time PinholeCameraModel::stamp() const
{
  assert( initialized() );
  return cam_info_.header.stamp;
}

IMAGE_GEOMETRY_PUBLIC
inline const sensor_msgs::msg::CameraInfo& PinholeCameraModel::cameraInfo() const  { return cam_info_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx33d& PinholeCameraModel::intrinsicMatrix() const  { return K_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Mat_<double>& PinholeCameraModel::distortionCoeffs() const { return D_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx33d& PinholeCameraModel::rotationMatrix() const   { return R_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx34d& PinholeCameraModel::projectionMatrix() const { return P_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx33d& PinholeCameraModel::fullIntrinsicMatrix() const  { return K_full_; }
IMAGE_GEOMETRY_PUBLIC
inline const cv::Matx34d& PinholeCameraModel::fullProjectionMatrix() const { return P_full_; }

IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::fx() const { return P_(0,0); }
IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::fy() const { return P_(1,1); }
IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::cx() const { return P_(0,2); }
IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::cy() const { return P_(1,2); }
IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::Tx() const { return P_(0,3); }
IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::Ty() const { return P_(1,3); }

IMAGE_GEOMETRY_PUBLIC
inline uint32_t PinholeCameraModel::binningX() const { return cam_info_.binning_x; }
IMAGE_GEOMETRY_PUBLIC
inline uint32_t PinholeCameraModel::binningY() const { return cam_info_.binning_y; }

IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::getDeltaU(double deltaX, double Z) const
{
  assert( initialized() );
  return fx() * deltaX / Z;
}

IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::getDeltaV(double deltaY, double Z) const
{
  assert( initialized() );
  return fy() * deltaY / Z;
}

IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::getDeltaX(double deltaU, double Z) const
{
  assert( initialized() );
  return Z * deltaU / fx();
}

IMAGE_GEOMETRY_PUBLIC
inline double PinholeCameraModel::getDeltaY(double deltaV, double Z) const
{
  assert( initialized() );
  return Z * deltaV / fy();
}

} //namespace image_geometry

#endif
