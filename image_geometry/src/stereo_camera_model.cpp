#include "image_geometry/stereo_camera_model.h"
#include <stdexcept>

namespace image_geometry {

StereoCameraModel::StereoCameraModel()
  : initialized_(false),
    Q_(4, 4, 0.0)
{
  Q_(0,0) = Q_(1,1) = 1.0;
}

StereoCameraModel::StereoCameraModel(const StereoCameraModel& other)
  : initialized_(false),
    left_(other.left_), right_(other.right_),
    Q_(4, 4, 0.0)
{
  Q_(0,0) = Q_(1,1) = 1.0;
  if (other.initialized_) {
    updateQ();
    initialized_ = true;
  }
}

void StereoCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& left,
                                       const sensor_msgs::CameraInfo& right)
{
  left_.fromCameraInfo(left);
  right_.fromCameraInfo(right);

  assert( left_.tfFrame() == right_.tfFrame() );
  assert( left_.fx() == right_.fx() );
  assert( left_.fy() == right_.fy() );
  assert( left_.cy() == right_.cy() );
  assert( left_.cx() == right_.cx() );
  /// @todo Cx may differ for verged cameras, support that case.

  updateQ();

  initialized_ = true;
}

void StereoCameraModel::updateQ()
{
  // Update variable fields of reprojection matrix
  /*
    From Springer Handbook of Robotics, p. 524:
        [ Fx 0  Cx -FxTx ]
    P = [ 0  Fy Cy   0   ]
        [ 0  0  1    0   ]

        [ 1 0   0      -Cx      ]
    Q = [ 0 1   0      -Cy      ]
        [ 0 0   0       Fx      ]
        [ 0 0 -1/Tx (Cx-Cx')/Tx ]
    where primed parameters are from the left projection matrix, unprimed from the right.

    Disparity = x_left - x_right?
   */
  /// @todo This is not exactly what stereo_image_proc does... see StereoData::setReprojection.
  double Tx = baseline();
  Q_(3,2) = -1.0 / Tx;
  Q_(0,3) = -right_.cx();
  Q_(1,3) = -right_.cy();
  Q_(2,3) = right_.fx();
  Q_(3,3) = (right_.cx() - left_.cx()) / Tx;
}

void StereoCameraModel::projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const
{
  assert(initialized_);

  throw std::runtime_error("[image_geometry] StereoCameraModel::projectPixelTo3d is unimplemented.");
}

} //namespace image_geometry
