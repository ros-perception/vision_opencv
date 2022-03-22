#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/distortion_models.h>
#include <opencv2/calib3d/calib3d.hpp>
#ifdef BOOST_SHARED_PTR_HPP_INCLUDED
#include <boost/make_shared.hpp>
#endif

namespace image_geometry {

enum DistortionState { NONE, CALIBRATED, UNKNOWN };
enum DistortionModel { EQUIDISTANT, PLUMB_BOB_OR_RATIONAL_POLYNOMIAL, UNKNOWN_MODEL };

struct PinholeCameraModel::Cache
{
  DistortionState distortion_state;
  DistortionModel distortion_model;

  cv::Mat_<double> K_binned, P_binned; // Binning applied, but not cropping

  mutable bool full_maps_dirty;
  mutable cv::Mat full_map1, full_map2;

  mutable bool reduced_maps_dirty;
  mutable cv::Mat reduced_map1, reduced_map2;

  mutable bool unrectify_full_maps_dirty;
  mutable cv::Mat unrectify_full_map1, unrectify_full_map2;

  mutable bool unrectify_reduced_maps_dirty;
  mutable cv::Mat unrectify_reduced_map1, unrectify_reduced_map2;

  mutable bool rectified_roi_dirty;
  mutable cv::Rect rectified_roi;

  Cache()
    : distortion_state(UNKNOWN),
      distortion_model(UNKNOWN_MODEL),
      full_maps_dirty(true),
      reduced_maps_dirty(true),
      unrectify_full_maps_dirty(true),
      unrectify_reduced_maps_dirty(true),
      rectified_roi_dirty(true)
  {
  }
};

PinholeCameraModel::PinholeCameraModel()
{
}

PinholeCameraModel& PinholeCameraModel::operator=(const PinholeCameraModel& other)
{
  if (other.initialized())
    this->fromCameraInfo(other.cameraInfo());
  return *this;
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

// For std::vector
template<typename MatT>
bool updateMat(const MatT& new_mat, MatT& my_mat, cv::Mat_<double>& cv_mat, int rows, int cols)
{
  if ((my_mat == new_mat) && (my_mat.size() == cv_mat.rows*cv_mat.cols))
    return false;
  my_mat = new_mat;
  // D may be empty if camera is uncalibrated or distortion model is non-standard
  cv_mat = (my_mat.size() == 0) ? cv::Mat_<double>() : cv::Mat_<double>(rows, cols, &my_mat[0]);
  return true;
}

template<typename MatT, typename MatU>
bool updateMat(const MatT& new_mat, MatT& my_mat, MatU& cv_mat)
{
  if ((my_mat == new_mat) && (my_mat.size() == cv_mat.rows*cv_mat.cols))
    return false;
  my_mat = new_mat;
  // D may be empty if camera is uncalibrated or distortion model is non-standard
  cv_mat = MatU(&my_mat[0]);
  return true;
}

bool PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  // Create our repository of cached data (rectification maps, etc.)
  if (!cache_)
#ifdef BOOST_SHARED_PTR_HPP_INCLUDED
    cache_ = boost::make_shared<Cache>();
#else
    cache_ = std::make_shared<Cache>();
#endif

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
  bool full_dirty = false;
  full_dirty |= update(msg.height, cam_info_.height);
  full_dirty |= update(msg.width,  cam_info_.width);
  full_dirty |= update(msg.distortion_model, cam_info_.distortion_model);
  full_dirty |= updateMat(msg.D, cam_info_.D, D_, 1, msg.D.size());
  full_dirty |= updateMat(msg.K, cam_info_.K, K_full_);
  full_dirty |= updateMat(msg.R, cam_info_.R, R_);
  full_dirty |= updateMat(msg.P, cam_info_.P, P_full_);
  full_dirty |= update(binning_x, cam_info_.binning_x);
  full_dirty |= update(binning_y, cam_info_.binning_y);
  cache_->full_maps_dirty |= full_dirty;
  cache_->unrectify_full_maps_dirty |= full_dirty;

  // The reduced rectification maps are invalidated by any of the above or a
  // change in ROI.
  bool reduced_dirty = full_dirty;
  reduced_dirty |= update(roi.x_offset,   cam_info_.roi.x_offset);
  reduced_dirty |= update(roi.y_offset,   cam_info_.roi.y_offset);
  reduced_dirty |= update(roi.height,     cam_info_.roi.height);
  reduced_dirty |= update(roi.width,      cam_info_.roi.width);
  reduced_dirty |= update(roi.do_rectify, cam_info_.roi.do_rectify);
  cache_->reduced_maps_dirty |= reduced_dirty;
  cache_->reduced_maps_dirty |= cache_->full_maps_dirty;
  cache_->unrectify_reduced_maps_dirty |= reduced_dirty;
  cache_->unrectify_reduced_maps_dirty |= cache_->unrectify_full_maps_dirty;

  // As is the rectified ROI
  cache_->rectified_roi_dirty |= reduced_dirty;

  // Figure out how to handle the distortion
  if (cam_info_.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB ||
      cam_info_.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL ||
      cam_info_.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
    // If any distortion coefficient is non-zero, then need to apply the distortion
    cache_->distortion_state = NONE;
    for (size_t i = 0; i < cam_info_.D.size(); ++i)
    {
      if (cam_info_.D[i] != 0)
      {
        cache_->distortion_state = CALIBRATED;
        break;
      }
    }
  }
  else
    cache_->distortion_state = UNKNOWN;

  // Get the distortion model, if supported
  if (cam_info_.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB ||
      cam_info_.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    cache_->distortion_model = PLUMB_BOB_OR_RATIONAL_POLYNOMIAL;
  }
  else if(cam_info_.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
    cache_->distortion_model = EQUIDISTANT;
  }
  else
    cache_->distortion_model = UNKNOWN_MODEL;

  // If necessary, create new K_ and P_ adjusted for binning and ROI
  /// @todo Calculate and use rectified ROI
  bool adjust_binning = (binning_x > 1) || (binning_y > 1);
  bool adjust_roi = (roi.x_offset != 0) || (roi.y_offset != 0);

  if (!adjust_binning && !adjust_roi) {
    K_ = K_full_;
    P_ = P_full_;
  }
  else {
    K_ = K_full_;
    P_ = P_full_;

    // ROI is in full image coordinates, so change it first
    if (adjust_roi) {
      // Move principal point by the offset
      /// @todo Adjust P by rectified ROI instead
      K_(0,2) -= roi.x_offset;
      K_(1,2) -= roi.y_offset;
      P_(0,2) -= roi.x_offset;
      P_(1,2) -= roi.y_offset;
    }

    if (binning_x > 1) {
      double scale_x = 1.0 / binning_x;
      K_(0,0) *= scale_x;
      K_(0,2) *= scale_x;
      P_(0,0) *= scale_x;
      P_(0,2) *= scale_x;
      P_(0,3) *= scale_x;
    }
    if (binning_y > 1) {
      double scale_y = 1.0 / binning_y;
      K_(1,1) *= scale_y;
      K_(1,2) *= scale_y;
      P_(1,1) *= scale_y;
      P_(1,2) *= scale_y;
      P_(1,3) *= scale_y;
    }
  }

  return reduced_dirty;
}

bool PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  return fromCameraInfo(*msg);
}

cv::Size PinholeCameraModel::fullResolution() const
{
  assert( initialized() );
  return cv::Size(cam_info_.width, cam_info_.height);
}

cv::Size PinholeCameraModel::reducedResolution() const
{
  assert( initialized() );

  cv::Rect roi = rectifiedRoi();
  return cv::Size(roi.width / binningX(), roi.height / binningY());
}

cv::Point2d PinholeCameraModel::toFullResolution(const cv::Point2d& uv_reduced) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Point2d(uv_reduced.x * binningX() + roi.x,
                     uv_reduced.y * binningY() + roi.y);
}

cv::Rect PinholeCameraModel::toFullResolution(const cv::Rect& roi_reduced) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Rect(roi_reduced.x * binningX() + roi.x,
                  roi_reduced.y * binningY() + roi.y,
                  roi_reduced.width  * binningX(),
                  roi_reduced.height * binningY());
}

cv::Point2d PinholeCameraModel::toReducedResolution(const cv::Point2d& uv_full) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Point2d((uv_full.x - roi.x) / binningX(),
                     (uv_full.y - roi.y) / binningY());
}

cv::Rect PinholeCameraModel::toReducedResolution(const cv::Rect& roi_full) const
{
  cv::Rect roi = rectifiedRoi();
  return cv::Rect((roi_full.x - roi.x) / binningX(),
                  (roi_full.y - roi.y) / binningY(),
                  roi_full.width  / binningX(),
                  roi_full.height / binningY());
}

cv::Rect PinholeCameraModel::rawRoi() const
{
  assert( initialized() );

  return cv::Rect(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                  cam_info_.roi.width, cam_info_.roi.height);
}

cv::Rect PinholeCameraModel::rectifiedRoi() const
{
  assert( initialized() );

  if (cache_->rectified_roi_dirty)
  {
    if (!cam_info_.roi.do_rectify)
      cache_->rectified_roi = rawRoi();
    else
      cache_->rectified_roi = rectifyRoi(rawRoi());
    cache_->rectified_roi_dirty = false;
  }
  return cache_->rectified_roi;
}

cv::Point2d PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz) const
{
  assert( initialized() );
  assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane

  // [U V W]^T = P * [X Y Z 1]^T
  // u = U/W
  // v = V/W
  cv::Point2d uv_rect;
  uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
  uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
  return uv_rect;
}

cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect) const
{
  return projectPixelTo3dRay(uv_rect, P_);
}

cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, const cv::Matx34d& P) const
{
  assert( initialized() );

  const double& fx = P(0,0);
  const double& fy = P(1,1);
  const double& cx = P(0,2);
  const double& cy = P(1,2);
  const double& Tx = P(0,3);
  const double& Ty = P(1,3);

  cv::Point3d ray;
  ray.x = (uv_rect.x - cx - Tx) / fx;
  ray.y = (uv_rect.y - cy - Ty) / fy;
  ray.z = 1.0;
  return ray;
}

void PinholeCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, int interpolation) const
{
  assert( initialized() );

  switch (cache_->distortion_state) {
    case NONE:
      raw.copyTo(rectified);
      break;
    case CALIBRATED:
      initRectificationMaps();
      if (raw.depth() == CV_32F || raw.depth() == CV_64F)
      {
        cv::remap(raw, rectified, cache_->reduced_map1, cache_->reduced_map2, interpolation, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
      }
      else {
        cv::remap(raw, rectified, cache_->reduced_map1, cache_->reduced_map2, interpolation);
      }
      break;
    default:
      assert(cache_->distortion_state == UNKNOWN);
      throw Exception("Cannot call rectifyImage when distortion is unknown.");
  }
}

void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const
{
  assert( initialized() );

  switch (cache_->distortion_state) {
    case NONE:
      rectified.copyTo(raw);
      break;
    case CALIBRATED:
      initUnrectificationMaps();
      if (rectified.depth() == CV_32F || rectified.depth() == CV_64F)
      {
        cv::remap(rectified, raw, cache_->unrectify_reduced_map1, cache_->unrectify_reduced_map2, interpolation, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
      }
      else {
        cv::remap(rectified, raw, cache_->unrectify_reduced_map1, cache_->unrectify_reduced_map2, interpolation);
      }
      break;
    default:
      assert(cache_->distortion_state == UNKNOWN);
      throw Exception("Cannot call rectifyImage when distortion is unknown.");
  }
}

cv::Point2d PinholeCameraModel::rectifyPoint(const cv::Point2d& uv_raw) const
{
  return rectifyPoint(uv_raw, K_, P_);
}

cv::Point2d PinholeCameraModel::rectifyPoint(const cv::Point2d& uv_raw, const cv::Matx33d& K, const cv::Matx34d& P) const
{
  assert( initialized() );

  if (cache_->distortion_state == NONE)
    return uv_raw;
  if (cache_->distortion_state == UNKNOWN)
    throw Exception("Cannot call rectifyPoint when distortion is unknown.");
  assert(cache_->distortion_state == CALIBRATED);

  /// @todo cv::undistortPoints requires the point data to be float, should allow double
  cv::Point2f raw32 = uv_raw, rect32;
  const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
  cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);

  switch (cache_->distortion_model) {
    case PLUMB_BOB_OR_RATIONAL_POLYNOMIAL:
      cv::undistortPoints(src_pt, dst_pt, K, D_, R_, P);
      break;
    case EQUIDISTANT:
      cv::fisheye::undistortPoints(src_pt, dst_pt, K, D_, R_, P);
      break;
    default:
      assert(cache_->distortion_model == UNKNOWN_MODEL);
      throw Exception("Wrong distortion model. Supported models: PLUMB_BOB, RATIONAL_POLYNOMIAL and EQUIDISTANT.");
  }
  return rect32;
}

cv::Point2d PinholeCameraModel::unrectifyPoint(const cv::Point2d& uv_rect) const
{
  return unrectifyPoint(uv_rect, K_, P_);
}

cv::Point2d PinholeCameraModel::unrectifyPoint(const cv::Point2d& uv_rect, const cv::Matx33d& K, const cv::Matx34d& P) const
{
  assert( initialized() );

  if (cache_->distortion_state == NONE)
    return uv_rect;
  if (cache_->distortion_state == UNKNOWN)
    throw Exception("Cannot call unrectifyPoint when distortion is unknown.");
  assert(cache_->distortion_state == CALIBRATED);

  // Convert to a ray
  cv::Point3d ray = projectPixelTo3dRay(uv_rect, P);

  // Project the ray on the image
  cv::Mat r_vec, t_vec = cv::Mat_<double>::zeros(3, 1);
  cv::Rodrigues(R_.t(), r_vec);
  std::vector<cv::Point2d> image_point;

  switch (cache_->distortion_model) {
    case PLUMB_BOB_OR_RATIONAL_POLYNOMIAL:
      cv::projectPoints(std::vector<cv::Point3d>(1, ray), r_vec, t_vec, K, D_, image_point);
      break;
    case EQUIDISTANT:
      cv::fisheye::projectPoints(std::vector<cv::Point3d>(1, ray), image_point, r_vec, t_vec, K, D_);
      break;
    default:
      assert(cache_->distortion_model == UNKNOWN_MODEL);
      throw Exception("Wrong distortion model. Supported models: PLUMB_BOB, RATIONAL_POLYNOMIAL and EQUIDISTANT.");
  }

  return image_point[0];
}

cv::Rect PinholeCameraModel::rectifyRoi(const cv::Rect& roi_raw) const
{
  assert( initialized() );

  /// @todo Actually implement "best fit" as described by REP 104.

  // For now, just unrectify the four corners and take the bounding box.
  // Since ROI is specified in unbinned coordinates (see REP-104), this has to use K_full_ and P_full_.
  cv::Point2d rect_tl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y), K_full_, P_full_);
  cv::Point2d rect_tr = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width, roi_raw.y), K_full_, P_full_);
  cv::Point2d rect_br = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width,
                                                 roi_raw.y + roi_raw.height), K_full_, P_full_);
  cv::Point2d rect_bl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y + roi_raw.height), K_full_, P_full_);

  cv::Point roi_tl(std::ceil (std::min(rect_tl.x, rect_bl.x)),
                   std::ceil (std::min(rect_tl.y, rect_tr.y)));
  cv::Point roi_br(std::floor(std::max(rect_tr.x, rect_br.x)),
                   std::floor(std::max(rect_bl.y, rect_br.y)));

  return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

cv::Rect PinholeCameraModel::unrectifyRoi(const cv::Rect& roi_rect) const
{
  assert( initialized() );

  /// @todo Actually implement "best fit" as described by REP 104.

  // For now, just unrectify the four corners and take the bounding box.
  cv::Point2d raw_tl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y));
  cv::Point2d raw_tr = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width, roi_rect.y));
  cv::Point2d raw_br = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width,
                                                  roi_rect.y + roi_rect.height));
  cv::Point2d raw_bl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y + roi_rect.height));

  cv::Point roi_tl(std::floor(std::min(raw_tl.x, raw_bl.x)),
                   std::floor(std::min(raw_tl.y, raw_tr.y)));
  cv::Point roi_br(std::ceil (std::max(raw_tr.x, raw_br.x)),
                   std::ceil (std::max(raw_bl.y, raw_br.y)));

  return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

void PinholeCameraModel::initRectificationMaps() const
{
  /// @todo For large binning settings, can drop extra rows/cols at bottom/right boundary.
  /// Make sure we're handling that 100% correctly.

  if (cache_->full_maps_dirty) {
    // Create the full-size map at the binned resolution
    /// @todo Should binned resolution, K, P be part of public API?
    cv::Size binned_resolution = fullResolution();
    binned_resolution.width  /= binningX();
    binned_resolution.height /= binningY();

    cv::Matx33d K_binned;
    cv::Matx34d P_binned;
    if (binningX() == 1 && binningY() == 1) {
      K_binned = K_full_;
      P_binned = P_full_;
    }
    else {
      K_binned = K_full_;
      P_binned = P_full_;
      if (binningX() > 1) {
        double scale_x = 1.0 / binningX();
        K_binned(0,0) *= scale_x;
        K_binned(0,2) *= scale_x;
        P_binned(0,0) *= scale_x;
        P_binned(0,2) *= scale_x;
        P_binned(0,3) *= scale_x;
      }
      if (binningY() > 1) {
        double scale_y = 1.0 / binningY();
        K_binned(1,1) *= scale_y;
        K_binned(1,2) *= scale_y;
        P_binned(1,1) *= scale_y;
        P_binned(1,2) *= scale_y;
        P_binned(1,3) *= scale_y;
      }
    }

    switch (cache_->distortion_model) {
      case PLUMB_BOB_OR_RATIONAL_POLYNOMIAL:
        // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
        cv::initUndistortRectifyMap(K_binned, D_, R_, P_binned, binned_resolution,
                                    CV_16SC2, cache_->full_map1, cache_->full_map2);
        break;
      case EQUIDISTANT:
        cv::fisheye::initUndistortRectifyMap(K_binned,D_, R_, P_binned, binned_resolution,
                                             CV_16SC2, cache_->full_map1, cache_->full_map2);
        break;
      default:
        assert(cache_->distortion_model == UNKNOWN_MODEL);
        throw Exception("Wrong distortion model. Supported models: PLUMB_BOB, RATIONAL_POLYNOMIAL and EQUIDISTANT.");
    }
    cache_->full_maps_dirty = false;
  }

  if (cache_->reduced_maps_dirty) {
    /// @todo Use rectified ROI
    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                 cam_info_.roi.width, cam_info_.roi.height);
    if (roi.x != 0 || roi.y != 0 ||
        (roi.height != 0 && roi.height != (int)cam_info_.height) ||
        (roi.width != 0 && roi.width  != (int)cam_info_.width)) {

      // map1 contains integer (x,y) offsets, which we adjust by the ROI offset
      // map2 contains LUT index for subpixel interpolation, which we can leave as-is
      roi.x /= binningX();
      roi.y /= binningY();
      roi.width  /= binningX();
      roi.height /= binningY();
      cache_->reduced_map1 = cache_->full_map1(roi) - cv::Scalar(roi.x, roi.y);
      cache_->reduced_map2 = cache_->full_map2(roi);
    }
    else {
      // Otherwise we're rectifying the full image
      cache_->reduced_map1 = cache_->full_map1;
      cache_->reduced_map2 = cache_->full_map2;
    }
    cache_->reduced_maps_dirty = false;
  }
}

void PinholeCameraModel::initUnrectificationMaps() const
{
  /// @todo For large binning settings, can drop extra rows/cols at bottom/right boundary.
  /// Make sure we're handling that 100% correctly.

  if (cache_->unrectify_full_maps_dirty) {
    // Create the full-size map at the binned resolution
    /// @todo Should binned resolution, K, P be part of public API?
    cv::Size binned_resolution = fullResolution();
    binned_resolution.width  /= binningX();
    binned_resolution.height /= binningY();

    cv::Matx33d K_binned;
    cv::Matx34d P_binned;
    if (binningX() == 1 && binningY() == 1) {
      K_binned = K_full_;
      P_binned = P_full_;
    }
    else {
      K_binned = K_full_;
      P_binned = P_full_;
      if (binningX() > 1) {
        double scale_x = 1.0 / binningX();
        K_binned(0,0) *= scale_x;
        K_binned(0,2) *= scale_x;
        P_binned(0,0) *= scale_x;
        P_binned(0,2) *= scale_x;
        P_binned(0,3) *= scale_x;
      }
      if (binningY() > 1) {
        double scale_y = 1.0 / binningY();
        K_binned(1,1) *= scale_y;
        K_binned(1,2) *= scale_y;
        P_binned(1,1) *= scale_y;
        P_binned(1,2) *= scale_y;
        P_binned(1,3) *= scale_y;
      }
    }

    cv::Mat float_map_x(binned_resolution.height, binned_resolution.width, CV_32FC1);
    cv::Mat float_map_y(binned_resolution.height, binned_resolution.width, CV_32FC1);
    for (size_t x = 0; x < binned_resolution.width; x++) {
      for (size_t y = 0; y < binned_resolution.height; y++) {
        cv::Point2f uv_raw(x, y), uv_rect;
        uv_rect = rectifyPoint(uv_raw, K_binned, P_binned);
        float_map_x.at<float>(y, x) = uv_rect.x;
        float_map_y.at<float>(y, x) = uv_rect.y;
      }
    }
    // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
    convertMaps(float_map_x, float_map_y, cache_->unrectify_full_map1, cache_->unrectify_full_map2, CV_16SC2);
    cache_->unrectify_full_maps_dirty = false;
  }

  if (cache_->unrectify_reduced_maps_dirty) {
    /// @todo Use rectified ROI
    cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
                 cam_info_.roi.width, cam_info_.roi.height);
    if (roi.x != 0 || roi.y != 0 ||
        (roi.height != 0 && roi.height != (int)cam_info_.height) ||
        (roi.width != 0 && roi.width  != (int)cam_info_.width)) {

      // map1 contains integer (x,y) offsets, which we adjust by the ROI offset
      // map2 contains LUT index for subpixel interpolation, which we can leave as-is
      roi.x /= binningX();
      roi.y /= binningY();
      roi.width  /= binningX();
      roi.height /= binningY();
      cache_->unrectify_reduced_map1 = cache_->unrectify_full_map1(roi) - cv::Scalar(roi.x, roi.y);
      cache_->unrectify_reduced_map2 = cache_->unrectify_full_map2(roi);
    }
    else {
      // Otherwise we're rectifying the full image
      cache_->unrectify_reduced_map1 = cache_->unrectify_full_map1;
      cache_->unrectify_reduced_map2 = cache_->unrectify_full_map2;
    }
    cache_->unrectify_reduced_maps_dirty = false;
  }
}

} //namespace image_geometry
