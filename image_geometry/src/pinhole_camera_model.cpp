#include "image_geometry/pinhole_camera_model.h"
#include <stdexcept>

namespace image_geometry {

PinholeCameraModel::PinholeCameraModel()
  : initialized_(false),
    K_(3, 3, &cam_info_.K[0]),
    D_(1, 5, &cam_info_.D[0]),
    R_(3, 3, &cam_info_.R[0]),
    P_(3, 4, &cam_info_.P[0])
{
}

PinholeCameraModel::PinholeCameraModel(const PinholeCameraModel& other)
  : initialized_(false),
    K_(3, 3, &cam_info_.K[0]),
    D_(1, 5, &cam_info_.D[0]),
    R_(3, 3, &cam_info_.R[0]),
    P_(3, 4, &cam_info_.P[0])
{
  if (other.initialized_)
    fromCameraInfo(other.cam_info_);
}

void PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  // The full-res camera dimensions should never change.
  assert(!initialized_ || (msg.height == cam_info_.height && msg.width == cam_info_.width));

  cam_info_.height = msg.height;
  cam_info_.width  = msg.width;
  
  parameters_changed_ = !initialized_ ||
    msg.K != cam_info_.K || msg.D != cam_info_.D ||
    msg.R != cam_info_.R || msg.P != cam_info_.P;
  
  roi_changed_ = !initialized_ ||
    msg.roi.x_offset != cam_info_.roi.x_offset ||
    msg.roi.y_offset != cam_info_.roi.y_offset ||
    msg.roi.height   != cam_info_.roi.height ||
    msg.roi.width    != cam_info_.roi.width;

  cam_info_.header = msg.header;
  
  if (parameters_changed_) {
    // Being extra sure we don't reallocate memory and invalidate our CvMats
    std::copy(msg.K.begin(), msg.K.end(), cam_info_.K.begin());
    std::copy(msg.D.begin(), msg.D.end(), cam_info_.D.begin());
    std::copy(msg.R.begin(), msg.R.end(), cam_info_.R.begin());
    std::copy(msg.P.begin(), msg.P.end(), cam_info_.P.begin());

    has_distortion_ = cam_info_.D[0] != 0.0;
  }

  if (roi_changed_) {
    cam_info_.roi = msg.roi;
    has_roi_ = (cam_info_.roi.width != 0 && cam_info_.roi.width != cam_info_.width) ||
      (cam_info_.roi.height != 0 && cam_info_.roi.height != cam_info_.height);
    /// @todo Adjust principal point in K and P? Will that do the right thing?
  }

  initialized_ = true;
}

void PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  fromCameraInfo(*msg);
}

void PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz, cv::Point2d& uv_rect) const
{
  assert(initialized_);

  /// @todo Principal point not adjusted for ROI anywhere yet
  uv_rect.x = fx() * (xyz.x / xyz.z) + cx();
  uv_rect.y = fy() * (xyz.y / xyz.z) + cy();
}

void PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, cv::Point3d& ray) const
{
  assert(initialized_);

  ray.x = (uv_rect.x - cx()) / fx();
  ray.y = (uv_rect.y - cy()) / fy();
  double norm = std::sqrt(ray.x*ray.x + ray.y*ray.y + 1);
  ray.x /= norm;
  ray.y /= norm;
  ray.z = 1.0 / norm;
}

void PinholeCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, int interpolation) const
{
  assert(initialized_);

  if (has_distortion_) {
    initUndistortMaps();
    if (has_roi_)
      cv::remap(raw, rectified, roi_undistort_map_x_, roi_undistort_map_y_, interpolation);
    else
      cv::remap(raw, rectified, undistort_map_x_, undistort_map_y_, interpolation);
  }
  else {
    raw.copyTo(rectified);
  }
}

void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw) const
{
  assert(initialized_);

  throw std::runtime_error("[image_geometry] PinholeCameraModel::unrectifyImage is unimplemented.");
}

void PinholeCameraModel::rectifyPoint(const cv::Point2d& uv_raw, cv::Point2d& uv_rect) const
{
  assert(initialized_);

  const cv::Mat src_pt(1, 1, CV_64FC2, const_cast<double*>(&uv_raw.x));
  cv::Mat dst_pt(1, 1, CV_64FC2, &uv_rect.x);
  cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
}

void PinholeCameraModel::unrectifyPoint(const cv::Point2d& uv_rect, cv::Point2d& uv_raw) const
{
  assert(initialized_);

  throw std::runtime_error("[image_geometry] PinholeCameraModel::unrectifyPoint is unimplemented.");
  /// @todo Do reverse transformation, see projectPoints & cvInitUndistortRectifyMap docs
}

void PinholeCameraModel::initUndistortMaps() const
{
  if (parameters_changed_) {
    // m1type=CV_16SC2 to use fast fixed-point maps
    cv::initUndistortRectifyMap(K_, D_, R_, P_, cv::Size(width(), height()),
                                CV_16SC2, undistort_map_x_, undistort_map_y_);
  }

  if (has_roi_ && (parameters_changed_ || roi_changed_)) {
    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                 cam_info_.roi.height, cam_info_.roi.width);
    //roi_undistort_map_x_ = undistort_map_x_(roi) - cv::Scalar(roi.x);
    cv::subtract(undistort_map_x_(roi), cv::Scalar(roi.x), roi_undistort_map_x_);
    cv::subtract(undistort_map_y_(roi), cv::Scalar(roi.y), roi_undistort_map_y_);
  }
}

} //namespace image_geometry
