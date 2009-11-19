#include "image_geometry/pinhole_camera_model.h"

namespace image_geometry {

PinholeCameraModel::PinholeCameraModel()
  : initialized_(false)
{
  K_ = cvMat(3, 3, CV_64FC1, &cam_info_.K[0]);
  D_ = cvMat(1, 5, CV_64FC1, &cam_info_.D[0]);
  R_ = cvMat(3, 3, CV_64FC1, &cam_info_.R[0]);
  P_ = cvMat(3, 4, CV_64FC1, &cam_info_.P[0]);
}

PinholeCameraModel::PinholeCameraModel(const PinholeCameraModel& other)
{
  /// @todo Copy constructor
}

void PinholeCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& msg)
{
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

void PinholeCameraModel::project3dToPixel(const Point3d& xyz, Point2d& uv_rect) const
{
  assert(initialized_);
  
}

void PinholeCameraModel::projectPixelTo3dRay(const Point2d& uv_rect, Point3d& ray) const
{
  assert(initialized_);
  
}

void PinholeCameraModel::rectifyImage(const CvArr* raw, CvArr* rectified) const
{
  assert(initialized_);

  if (has_distortion_) {
    initUndistortMaps();
    IplImage *undistort_x, *undistort_y;
    if (has_roi_) {
      undistort_x = roi_undistort_map_x_.Ipl();
      undistort_y = roi_undistort_map_y_.Ipl();
    } else {
      undistort_x = undistort_map_x_.Ipl();
      undistort_y = undistort_map_y_.Ipl();
    }
    /// @todo Allow user to choose interpolation
    cvRemap(raw, rectified, undistort_x, undistort_y, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
  }
  else {
    cvCopy(raw, rectified);
  }
}

void PinholeCameraModel::unrectifyImage(const CvArr* rectified, CvArr* raw) const
{
  assert(initialized_);
  
}

void PinholeCameraModel::rectifyPoint(const Point2d& uv_raw, Point2d& uv_rect) const
{
  assert(initialized_);
  
}

void PinholeCameraModel::unrectifyPoint(const Point2d& uv_rect, Point2d& uv_raw) const
{
  assert(initialized_);
  
}

void PinholeCameraModel::initUndistortMaps() const
{
  if (parameters_changed_) {
    undistort_map_x_.Allocate(cam_info_.width, cam_info_.height);
    undistort_map_y_.Allocate(cam_info_.width, cam_info_.height);
    cvInitUndistortRectifyMap(&K_, &D_, &R_, &P_, undistort_map_x_.Ipl(), undistort_map_y_.Ipl());
  }

  if (has_roi_ && (parameters_changed_ || roi_changed_)) {
    uint32_t x = cam_info_.roi.x_offset;
    uint32_t y = cam_info_.roi.y_offset;
    uint32_t height = cam_info_.roi.height;
    uint32_t width = cam_info_.roi.width;
    roi_undistort_map_x_.Allocate(width, height);
    roi_undistort_map_y_.Allocate(width, height);
    cvSubS(undistort_map_x_.View(x, y, width, height).Ipl(), cvScalar(x), roi_undistort_map_x_.Ipl());
    cvSubS(undistort_map_y_.View(x, y, width, height).Ipl(), cvScalar(y), roi_undistort_map_y_.Ipl());
  }
}

} //namespace image_geometry
