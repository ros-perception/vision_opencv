#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <sensor_msgs/distortion_models.hpp>

#include "image_geometry/stereo_camera_model.hpp"

class StereoTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // Rectified projection matrices taken from a real stereo calibration (see test/directed.py).
    // The right principal point is shifted to also exercise verged cameras, where
    // the disparity is offset by (left.cx - right.cx).
    double P_left[] = {295.53402059708782, 0.0, 285.55760765075684, 0.0,
      0.0, 295.53402059708782, 223.29617881774902, 0.0,
      0.0, 0.0, 1.0, 0.0};
    double P_right[] = {295.53402059708782, 0.0, 290.0, -26.507895206214123,
      0.0, 295.53402059708782, 223.29617881774902, 0.0,
      0.0, 0.0, 1.0, 0.0};
    double K[] = {430.15433020105519, 0.0, 311.71339830549732,
      0.0, 430.60920415473657, 221.06824942698509,
      0.0, 0.0, 1.0};
    double R[] = {1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

    for (sensor_msgs::msg::CameraInfo * info : {&left_info_, &right_info_}) {
      info->header.frame_id = "left_camera";
      info->height = 480;
      info->width = 640;
      info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      info->d.assign(5, 0.0);
      std::copy(K, K + 9, info->k.begin());
      std::copy(R, R + 9, info->r.begin());
    }
    std::copy(P_left, P_left + 12, left_info_.p.begin());
    std::copy(P_right, P_right + 12, right_info_.p.begin());

    model_.fromCameraInfo(left_info_, right_info_);
  }

  sensor_msgs::msg::CameraInfo left_info_, right_info_;
  image_geometry::StereoCameraModel model_;
};

TEST_F(StereoTest, project3dToDisparityIsInverseOfProjectDisparityTo3d)
{
  for (double u : {16.0, 320.0, 624.0}) {
    for (double v : {16.0, 240.0, 464.0}) {
      for (float d : {1.0f, 4.5f, 9.0f, 30.0f}) {
        cv::Point3d xyz;
        model_.projectDisparityTo3d(cv::Point2d(u, v), d, xyz);

        cv::Point2d left_uv;
        float disparity = 0.0f;
        model_.project3dToDisparity(xyz, left_uv, disparity);

        EXPECT_NEAR(u, left_uv.x, 1e-6);
        EXPECT_NEAR(v, left_uv.y, 1e-6);
        EXPECT_NEAR(d, disparity, 1e-4);
      }
    }
  }
}

TEST_F(StereoTest, project3dToDisparityMatchesRightCamera)
{
  const cv::Point3d xyz(0.3, -0.2, 2.5);

  cv::Point2d left_uv;
  float disparity = 0.0f;
  model_.project3dToDisparity(xyz, left_uv, disparity);

  const cv::Point2d expected_left = model_.left().project3dToPixel(xyz);
  const cv::Point2d expected_right = model_.right().project3dToPixel(xyz);
  EXPECT_NEAR(expected_left.x, left_uv.x, 1e-9);
  EXPECT_NEAR(expected_left.y, left_uv.y, 1e-9);
  EXPECT_NEAR(expected_left.x - expected_right.x, disparity, 1e-4);
  EXPECT_NEAR(model_.getDisparity(xyz.z), disparity, 1e-4);
}

// Checks project3dToDisparity() against both cameras, getDisparity(), getZ() and
// projectDisparityTo3d() for the given point.
static void expectConsistent(
  const image_geometry::StereoCameraModel & model, const cv::Point3d & xyz)
{
  SCOPED_TRACE(testing::Message() << "xyz = " << xyz);

  cv::Point2d left_uv;
  float disparity = 0.0f;
  model.project3dToDisparity(xyz, left_uv, disparity);

  const cv::Point2d expected_left = model.left().project3dToPixel(xyz);
  const cv::Point2d expected_right = model.right().project3dToPixel(xyz);
  EXPECT_NEAR(expected_left.x, left_uv.x, 1e-9);
  EXPECT_NEAR(expected_left.y, left_uv.y, 1e-9);
  // The point is seen on the same row of both images, disparity pixels apart
  EXPECT_NEAR(expected_right.x, left_uv.x - disparity, 1e-4);
  EXPECT_NEAR(expected_right.y, left_uv.y, 1e-9);

  EXPECT_NEAR(model.getDisparity(xyz.z), disparity, 1e-4);
  EXPECT_NEAR(model.getZ(disparity), xyz.z, 1e-4 * xyz.z * xyz.z);

  cv::Point3d back;
  model.projectDisparityTo3d(left_uv, disparity, back);
  // The disparity is a float, so the depth is recovered less exactly the further it is
  const double tolerance = 1e-4 * xyz.z * xyz.z;
  EXPECT_NEAR(xyz.x, back.x, tolerance);
  EXPECT_NEAR(xyz.y, back.y, tolerance);
  EXPECT_NEAR(xyz.z, back.z, tolerance);
}

static const cv::Point3d kPoints[] = {
  {0.0, 0.0, 1.0},        // on the optical axis
  {0.3, -0.2, 2.5},
  {-0.7, 0.4, 0.8},       // left of and below the axis, close
  {1.5, 1.0, 12.0},       // far
  {-4.0, -3.0, 3.0}};     // projects outside of the image

TEST_F(StereoTest, project3dToDisparityVergedCameras)
{
  ASSERT_NE(model_.left().cx(), model_.right().cx());
  for (const cv::Point3d & xyz : kPoints) {
    expectConsistent(model_, xyz);
  }
}

// Without vergence the disparity is fx * baseline / z.
TEST_F(StereoTest, project3dToDisparityParallelCameras)
{
  right_info_.p[2] = left_info_.p[2];
  model_.fromCameraInfo(left_info_, right_info_);
  ASSERT_EQ(model_.left().cx(), model_.right().cx());

  for (const cv::Point3d & xyz : kPoints) {
    expectConsistent(model_, xyz);

    cv::Point2d left_uv;
    float disparity = 0.0f;
    model_.project3dToDisparity(xyz, left_uv, disparity);
    EXPECT_NEAR(model_.left().fx() * model_.baseline() / xyz.z, disparity, 1e-4);
  }
}

// The focal lengths of the two axes may differ.
TEST_F(StereoTest, project3dToDisparityDifferentFocalLengths)
{
  left_info_.p[5] = right_info_.p[5] = 310.0;
  model_.fromCameraInfo(left_info_, right_info_);
  ASSERT_NE(model_.left().fx(), model_.left().fy());

  for (const cv::Point3d & xyz : kPoints) {
    expectConsistent(model_, xyz);
  }
}

// Binning and a region of interest change the pixel coordinates and the disparity.
TEST_F(StereoTest, project3dToDisparityBinningAndRoi)
{
  const cv::Point3d xyz(0.3, -0.2, 2.5);
  cv::Point2d full_uv;
  float full_disparity = 0.0f;
  model_.project3dToDisparity(xyz, full_uv, full_disparity);

  for (sensor_msgs::msg::CameraInfo * info : {&left_info_, &right_info_}) {
    info->binning_x = 2;
    info->binning_y = 4;
    info->roi.x_offset = 40;
    info->roi.y_offset = 80;
    info->roi.width = 400;
    info->roi.height = 320;
  }
  model_.fromCameraInfo(left_info_, right_info_);

  for (const cv::Point3d & point : kPoints) {
    expectConsistent(model_, point);
  }

  cv::Point2d left_uv;
  float disparity = 0.0f;
  model_.project3dToDisparity(xyz, left_uv, disparity);
  EXPECT_NEAR((full_uv.x - 40) / 2, left_uv.x, 1e-9);
  EXPECT_NEAR((full_uv.y - 80) / 4, left_uv.y, 1e-9);
  // The disparity is horizontal: it scales with binning_x and is not shifted by the ROI
  EXPECT_NEAR(full_disparity / 2, disparity, 1e-4);
}

// The disparity only depends on the depth, and shrinks as the point moves away.
TEST_F(StereoTest, project3dToDisparityDependsOnDepthOnly)
{
  cv::Point2d left_uv;
  float near_disparity = 0.0f, far_disparity = 0.0f, shifted_disparity = 0.0f;
  model_.project3dToDisparity(cv::Point3d(0.3, -0.2, 2.0), left_uv, near_disparity);
  model_.project3dToDisparity(cv::Point3d(0.3, -0.2, 8.0), left_uv, far_disparity);
  model_.project3dToDisparity(cv::Point3d(-1.0, 0.9, 2.0), left_uv, shifted_disparity);

  EXPECT_GT(near_disparity, far_disparity);
  EXPECT_FLOAT_EQ(near_disparity, shifted_disparity);

  // At infinity only the offset of verged cameras is left
  float infinity_disparity = 0.0f;
  model_.project3dToDisparity(cv::Point3d(0.0, 0.0, 1e12), left_uv, infinity_disparity);
  EXPECT_NEAR(model_.left().cx() - model_.right().cx(), infinity_disparity, 1e-4);
}

// A point in the plane of the cameras is not visible: nothing finite comes back.
TEST_F(StereoTest, project3dToDisparityZeroDepth)
{
  cv::Point2d left_uv;
  float disparity = 0.0f;
  model_.project3dToDisparity(cv::Point3d(0.3, -0.2, 0.0), left_uv, disparity);
  EXPECT_TRUE(std::isinf(disparity));
  EXPECT_FALSE(std::isfinite(left_uv.x));
  EXPECT_FALSE(std::isfinite(left_uv.y));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
