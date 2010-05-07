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
  assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane
  /// @todo Principal point not adjusted for ROI anywhere yet

  uv_rect.x = (fx()*xyz.x + P_(0, 3)) / xyz.z + cx();
  uv_rect.y = (fy()*xyz.y + P_(1, 3)) / xyz.z + cy();
}

void PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, cv::Point3d& ray) const
{
  assert(initialized_);

  ray.x = (uv_rect.x - cx()) / fx();
  ray.y = (uv_rect.y - cy()) / fy();
  ray.z = 1.0;
  double norm = std::sqrt(ray.x*ray.x + ray.y*ray.y + 1);
  ray *= 1.0/norm;
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

void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const
{
  assert(initialized_);

  throw std::runtime_error("[image_geometry] PinholeCameraModel::unrectifyImage is unimplemented.");
  // Similar to rectifyImage, but need to build separate set of inverse maps (raw->rectified)...
  // - Build src_pt Mat with all the raw pixel coordinates (or do it one row at a time)
  // - Do cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_)
  // - Use convertMaps() to convert dst_pt to fast fixed-point maps
  // - cv::remap(rectified, raw, ...)
  // Need interpolation argument. Same caching behavior?
}

void PinholeCameraModel::rectifyPoint(const cv::Point2d& uv_raw, cv::Point2d& uv_rect) const
{
  assert(initialized_);

  // cv::undistortPoints requires the point data to be float
  cv::Point2f raw32 = uv_raw, rect32;
  const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
  cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
  cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
  uv_rect = rect32;
}

void PinholeCameraModel::unrectifyPoint(const cv::Point2d& uv_rect, cv::Point2d& uv_raw) const
{
  assert(initialized_);

  if (!has_distortion_) {
    uv_raw = uv_rect;
    return;
  }

  /// @todo Is there not an OpenCV call to do this directly?

  // Formulae from docs for cv::initUndistortRectifyMap,
  // http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html

  // x <- (u - c'x) / f'x
  // y <- (v - c'y) / f'y
  // c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3].
  double x = (uv_rect.x - cx()) / fx();
  double y = (uv_rect.y - cy()) / fy();
  // [X Y W]^T <- R^-1 * [x y 1]^T
  double X = R_(0,0)*x + R_(1,0)*y + R_(2,0);
  double Y = R_(0,1)*x + R_(1,1)*y + R_(2,1);
  double W = R_(0,2)*x + R_(1,2)*y + R_(2,2);
  // x' <- X/W, y' <- Y/W
  double xp = X / W;
  double yp = Y / W;
  // x'' <- x'(1+k1*r^2+k2*r^4+k3*r^6) + 2p1*x'*y' + p2(r^2+2x'^2)
  // y'' <- y'(1+k1*r^2+k2*r^4+k3*r^6) + p1(r^2+2y'^2) + 2p2*x'*y'
  // where r^2 = x'^2 + y'^2
  double r2 = xp*xp + yp*yp;
  double k1 = D_(0,0), k2 = D_(0,1), p1 = D_(0,2), p2 = D_(0,3), k3 = D_(0,4);
  double barrel_correction = 1 + k1*r2 + k2*(r2*r2) + k3*(r2*r2*r2);
  double xpp = xp*barrel_correction + 2*p1*(xp*yp) + p2*(r2+2*(xp*xp));
  double ypp = yp*barrel_correction + p1*(r2+2*(yp*yp)) + 2*p2*(xp*yp);
  // map_x(u,v) <- x''fx + cx
  // map_y(u,v) <- y''fy + cy
  // cx, fx, etc. come from original camera matrix K.
  uv_raw.x = xpp*K_(0,0) + K_(0,2);
  uv_raw.y = ypp*K_(1,1) + K_(1,2);
}

void PinholeCameraModel::initUndistortMaps() const
{
  if (undistort_map_x_.empty() || parameters_changed_) {
    // m1type=CV_16SC2 to use fast fixed-point maps
    cv::initUndistortRectifyMap(K_, D_, R_, P_, cv::Size(width(), height()),
                                CV_16SC2, undistort_map_x_, undistort_map_y_);
  }

  if (has_roi_ && (roi_undistort_map_x_.empty() || parameters_changed_ || roi_changed_)) {
    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                 cam_info_.roi.height, cam_info_.roi.width);
    //roi_undistort_map_x_ = undistort_map_x_(roi) - cv::Scalar(roi.x);
    cv::subtract(undistort_map_x_(roi), cv::Scalar(roi.x), roi_undistort_map_x_);
    cv::subtract(undistort_map_y_(roi), cv::Scalar(roi.y), roi_undistort_map_y_);
  }

  parameters_changed_ = roi_changed_ = false;
}

} //namespace image_geometry
