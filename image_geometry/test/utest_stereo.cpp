#include "image_geometry/stereo_camera_model.hpp"
#include <sensor_msgs/distortion_models.hpp>
#include <gtest/gtest.h>

#include <algorithm>

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
