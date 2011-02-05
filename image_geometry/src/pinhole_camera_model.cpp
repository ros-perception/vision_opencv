#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/distortion_models.h>
#include <boost/make_shared.hpp>
#include <stdexcept>

namespace image_geometry {

enum DistortionState { NONE, CALIBRATED, UNKNOWN };

struct PinholeCameraModel::Cache
{
  DistortionState distortion_state;
  mutable bool full_maps_dirty, current_maps_dirty;
  mutable cv::Mat full_map1, full_map2;
  mutable cv::Mat current_map1, current_map2;

  Cache() : full_maps_dirty(true), current_maps_dirty(true) {}
};

PinholeCameraModel::PinholeCameraModel()
{
}

PinholeCameraModel::PinholeCameraModel(const PinholeCameraModel& other)
{
  if (other.initialized())
    fromCameraInfo(other.cam_info_);
}

// For uint32_t, string, bool...
template<typename T>
bool update(const T& new_val, T& my_val)
{
  if (my_val == new_val)
    return false;
  my_val = new_val;
  return true;
}

// For boost::array, std::vector
template<typename MatT>
bool updateMat(const MatT& new_mat, MatT& my_mat, cv::Mat_<double>& cv_mat, int rows, int cols)
{
  if (my_mat == new_mat)
    return false;
  my_mat = new_mat;
  // D may be empty if camera is uncalibrated or distortion model is non-standard
  cv_mat = (my_mat.size() == 0) ? cv::Mat_<double>() : cv::Mat_<double>(rows, cols, &my_mat[0]);
  return true;
}

void PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  // Create our repository of cached data (rectification maps, etc.)
  if (!cache_)
    cache_ = boost::make_shared<Cache>();
  
  // Binning = 0 is considered the same as binning = 1 (no binning).
  uint32_t binning_x = msg.binning_x ? msg.binning_x : 1;
  uint32_t binning_y = msg.binning_y ? msg.binning_y : 1;

  // ROI all zeros is considered the same as full resolution.
  sensor_msgs::RegionOfInterest roi = msg.roi;
  if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
    roi.width  = msg.width;
    roi.height = msg.height;
  }

  // Update time stamp (and frame_id if that changes for some reason)
  cam_info_.header = msg.header;
  
  // Update any parameters that have changed. The full rectification maps are
  // invalidated by any change in the calibration parameters OR binning.
  bool &full_dirty = cache_->full_maps_dirty;
  full_dirty |= update(msg.height, cam_info_.height);
  full_dirty |= update(msg.width,  cam_info_.width);
  full_dirty |= update(msg.distortion_model, cam_info_.distortion_model);
  full_dirty |= updateMat(msg.D, cam_info_.D, D_, 1, msg.D.size());
  full_dirty |= updateMat(msg.K, cam_info_.K, K_full_, 3, 3);
  full_dirty |= updateMat(msg.R, cam_info_.R, R_, 3, 3);
  full_dirty |= updateMat(msg.P, cam_info_.P, P_full_, 3, 4);
  full_dirty |= update(binning_x, cam_info_.binning_x);
  full_dirty |= update(binning_y, cam_info_.binning_y);

  // The current rectification maps are invalidated by any of the above or a
  // change in ROI.
  bool &current_dirty = cache_->current_maps_dirty;
  current_dirty  = full_dirty;
  current_dirty |= update(roi.x_offset,   cam_info_.roi.x_offset);
  current_dirty |= update(roi.y_offset,   cam_info_.roi.y_offset);
  current_dirty |= update(roi.height,     cam_info_.roi.height);
  current_dirty |= update(roi.width,      cam_info_.roi.width);
  current_dirty |= update(roi.do_rectify, cam_info_.roi.do_rectify);

  // Figure out how to handle the distortion
  if (cam_info_.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB ||
      cam_info_.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    cache_->distortion_state = (cam_info_.D[0] == 0.0) ? NONE : CALIBRATED;
  }
  else
    cache_->distortion_state = UNKNOWN;

  // If necessary, create new K_ and P_ adjusted for binning and ROI
  /// @todo Calculate and use rectified ROI
  bool adjust_binning = (binning_x > 1) || (binning_y > 1);
  bool adjust_roi = (roi.x_offset != 0) || (roi.y_offset != 0);

  if (!adjust_binning && !adjust_roi) {
    K_ = K_full_;
    P_ = P_full_;
  }
  else {
    K_full_.copyTo(K_);
    P_full_.copyTo(P_);

    // ROI is in full image coordinates, so change it first
    if (adjust_roi) {
      // Move principal point by the offset
      K_(0,2) -= roi.x_offset;
      K_(1,2) -= roi.y_offset;
      P_(0,2) -= roi.x_offset;
      P_(1,2) -= roi.y_offset;
    }

    if (adjust_binning) {
      // Rescale all values
      double scale_x = 1.0 / binning_x;
      double scale_y = 1.0 / binning_y;
      K_(0,0) *= scale_x;
      K_(0,2) *= scale_x;
      K_(1,1) *= scale_y;
      K_(1,2) *= scale_y;
      P_(0,0) *= scale_x;
      P_(0,2) *= scale_x;
      P_(0,3) *= scale_x;
      P_(1,1) *= scale_y;
      P_(1,2) *= scale_y;
      P_(1,3) *= scale_y;
    }
  }
}

void PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  fromCameraInfo(*msg);
}

void PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz, cv::Point2d& uv_rect) const
{
  assert( initialized() );
  assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane

  // [U V W]^T = P * [X Y Z 1]^T
  // u = U/W
  // v = V/W
  uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
  uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
}

void PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, cv::Point3d& ray) const
{
  assert( initialized() );

  ray.x = (uv_rect.x - cx() - Tx()) / fx();
  ray.y = (uv_rect.y - cy() - Ty()) / fy();
  ray.z = 1.0;
}

void PinholeCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, int interpolation) const
{
  assert( initialized() );

  switch (cache_->distortion_state) {
    case NONE:
      raw.copyTo(rectified);
      break;
    case CALIBRATED:
      initUndistortMaps();
      cv::remap(raw, rectified, cache_->current_map1, cache_->current_map2, interpolation);
      break;
    default:
      assert(cache_->distortion_state == UNKNOWN);
      throw std::runtime_error("Cannot call rectifyImage when distortion is unknown.");
  }
}

void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const
{
  assert( initialized() );

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
  assert( initialized() );

  if (cache_->distortion_state == NONE) {
    uv_rect = uv_raw;
    return;
  }
  if (cache_->distortion_state == UNKNOWN) {
    throw std::runtime_error("Cannot call rectifyPoint when distortion is unknown.");
  }
  assert(cache_->distortion_state == CALIBRATED);

  /// @todo cv::undistortPoints requires the point data to be float, should allow double
  cv::Point2f raw32 = uv_raw, rect32;
  const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
  cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
  cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
  uv_rect = rect32;
}

void PinholeCameraModel::unrectifyPoint(const cv::Point2d& uv_rect, cv::Point2d& uv_raw) const
{
  assert( initialized() );

  if (cache_->distortion_state == NONE) {
    uv_raw = uv_rect;
    return;
  }
  if (cache_->distortion_state == UNKNOWN) {
    throw std::runtime_error("Cannot call unrectifyPoint when distortion is unknown.");
  }
  assert(cache_->distortion_state == CALIBRATED);

  /// @todo Make this just call projectPixelTo3dRay followed by cv::projectPoints. But
  /// cv::projectPoints requires 32-bit float, which is annoying.

  // Formulae from docs for cv::initUndistortRectifyMap,
  // http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html

  // x <- (u - c'x) / f'x
  // y <- (v - c'y) / f'y
  // c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3].
  double x = (uv_rect.x - cx() - Tx()) / fx();
  double y = (uv_rect.y - cy() - Ty()) / fy();
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
  double r4 = r2*r2;
  double r6 = r4*r2;
  double a1 = 2*xp*yp;
  double k1 = D_(0,0), k2 = D_(0,1), p1 = D_(0,2), p2 = D_(0,3), k3 = D_(0,4);
  double barrel_correction = 1 + k1*r2 + k2*r4 + k3*r6;
  if (D_.cols == 8)
    barrel_correction /= (1.0 + D_(0,5)*r2 + D_(0,6)*r4 + D_(0,7)*r6);
  double xpp = xp*barrel_correction + p1*a1 + p2*(r2+2*(xp*xp));
  double ypp = yp*barrel_correction + p1*(r2+2*(yp*yp)) + p2*a1;
  // map_x(u,v) <- x''fx + cx
  // map_y(u,v) <- y''fy + cy
  // cx, fx, etc. come from original camera matrix K.
  uv_raw.x = xpp*K_(0,0) + K_(0,2);
  uv_raw.y = ypp*K_(1,1) + K_(1,2);
}

void PinholeCameraModel::initUndistortMaps() const
{
  if (cache_->full_maps_dirty) {
    // m1type=CV_16SC2 to use fast fixed-point maps
    cv::initUndistortRectifyMap(K_full_, D_, R_, P_full_, cv::Size(width(), height()), /// @todo fullResolution()
                                CV_16SC2, cache_->full_map1, cache_->full_map2);
  }

  if (cache_->current_maps_dirty) {
    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                 cam_info_.roi.width, cam_info_.roi.height);
    if (roi.x != 0 || roi.y != 0 ||
        roi.height != (int)cam_info_.height ||
        roi.width != (int)cam_info_.height) {
      
      //roi_undistort_map_x_ = undistort_map_x_(roi) - cv::Scalar(roi.x);
      //cv::subtract(undistort_map_x_(roi), cv::Scalar(roi.x), roi_undistort_map_x_);
      //cv::subtract(undistort_map_y_(roi), cv::Scalar(roi.y), roi_undistort_map_y_);
      cache_->current_map1 = cache_->full_map1(roi) - cv::Scalar(roi.x, roi.y);
      cache_->current_map2 = cache_->full_map2(roi);
    }
    else {
      // Otherwise we're rectifying the full image
      cache_->current_map1 = cache_->full_map1;
      cache_->current_map2 = cache_->full_map2;
    }
  }
}

} //namespace image_geometry
