#ifndef IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_PINHOLE_CAMERA_MODEL_H

#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <stdexcept>

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
   * \brief Get the time stamp associated with this camera model.
   */
  ros::Time stamp() const;
  
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
  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw,
                      int interpolation = CV_INTER_LINEAR) const;

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
   * \brief Returns the x-translation term of the projection matrix.
   */
  double Tx() const;

  /**
   * \brief Returns the y-translation term of the projection matrix.
   */
  double Ty() const;

  /// @todo Deprecate height(), width()
  /**
   * \brief Returns the image height.
   */
  uint32_t height() const;

  /**
   * \brief Returns the image width.
   */
  uint32_t width() const;

  /**
   * \brief Returns the number of columns in each bin.
   */
  uint32_t binningX() const;

  /**
   * \brief Returns the number of rows in each bin.
   */
  uint32_t binningY() const;
  
  /**
   * \brief Compute delta u, given Z and delta X in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaX().
   *
   * \param deltaX Delta X, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaU(double deltaX, double Z) const;

  /**
   * \brief Compute delta v, given Z and delta Y in Cartesian space.
   *
   * For given Z, this is the inverse of getDeltaY().
   *
   * \param deltaY Delta Y, in Cartesian space
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaV(double deltaY, double Z) const;

  /**
   * \brief Compute delta X, given Z in Cartesian space and delta u in pixels.
   *
   * For given Z, this is the inverse of getDeltaU().
   *
   * \param deltaU Delta u, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaX(double deltaU, double Z) const;

  /**
   * \brief Compute delta Y, given Z in Cartesian space and delta v in pixels.
   *
   * For given Z, this is the inverse of getDeltaV().
   *
   * \param deltaV Delta v, in pixels
   * \param Z      Z (depth), in Cartesian space
   */
  double getDeltaY(double deltaV, double Z) const;
  
protected:
  sensor_msgs::CameraInfo cam_info_;
  cv::Mat_<double> D_, R_;           // Unaffected by binning, ROI
  cv::Mat_<double> K_, P_;           // Describe current image (includes binning, ROI)
  cv::Mat_<double> K_full_, P_full_; // Describe full-res image, needed for full maps

  // Use PIMPL here so we can change internals in patch updates if needed
  struct Cache;
  boost::shared_ptr<Cache> cache_; // Holds cached data for internal use

  void initUndistortMaps() const;

  bool initialized() const { return cache_; }
};


/* Trivial inline functions */
inline std::string PinholeCameraModel::tfFrame() const
{
  assert( initialized() );
  return cam_info_.header.frame_id;
}

inline ros::Time PinholeCameraModel::stamp() const
{
  assert( initialized() );
  return cam_info_.header.stamp;
}

inline const cv::Mat_<double>& PinholeCameraModel::intrinsicMatrix() const  { return K_; }
inline const cv::Mat_<double>& PinholeCameraModel::distortionCoeffs() const { return D_; }
inline const cv::Mat_<double>& PinholeCameraModel::rotationMatrix() const   { return R_; }
inline const cv::Mat_<double>& PinholeCameraModel::projectionMatrix() const { return P_; }

inline double PinholeCameraModel::fx() const { return P_(0,0); }
inline double PinholeCameraModel::fy() const { return P_(1,1); }
inline double PinholeCameraModel::cx() const { return P_(0,2); }
inline double PinholeCameraModel::cy() const { return P_(1,2); }
inline double PinholeCameraModel::Tx() const { return P_(0,3); }
inline double PinholeCameraModel::Ty() const { return P_(1,3); }
inline uint32_t PinholeCameraModel::height() const { return cam_info_.height; }
inline uint32_t PinholeCameraModel::width() const  { return cam_info_.width; }

inline uint32_t PinholeCameraModel::binningX() const { return cam_info_.binning_x ? cam_info_.binning_x : 1; }
inline uint32_t PinholeCameraModel::binningY() const { return cam_info_.binning_y ? cam_info_.binning_y : 1; }

inline double PinholeCameraModel::getDeltaU(double deltaX, double Z) const
{
  assert( initialized() );
  return fx() * deltaX / Z;
}

inline double PinholeCameraModel::getDeltaV(double deltaY, double Z) const
{
  assert( initialized() );
  return fy() * deltaY / Z;
}

inline double PinholeCameraModel::getDeltaX(double deltaU, double Z) const
{
  assert( initialized() );
  return Z * deltaU / fx();
}

inline double PinholeCameraModel::getDeltaY(double deltaV, double Z) const
{
  assert( initialized() );
  return Z * deltaV / fy();
}

} //namespace image_geometry

#endif
