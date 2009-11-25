#include "image_geometry/pinhole_camera_model.h"
#include <gtest/gtest.h>

class PinholeTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    // These parameters taken from a real camera calibration
    double D[] = {-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05, -0.00044776712298447841, 0.0};
    double K[] = {430.15433020105519,                0.0, 311.71339830549732,
                                 0.0, 430.60920415473657, 221.06824942698509,
                                 0.0,                0.0,                1.0};
    double R[] = {0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
                  -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
                  -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516};
    double P[] = {295.53402059708782, 0.0, 285.55760765075684, 0.0,
                  0.0, 295.53402059708782, 223.29617881774902, 0.0,
                  0.0, 0.0, 1.0, 0.0};

    cam_info_.header.frame_id = "tf_frame";
    cam_info_.height = 480;
    cam_info_.width  = 640;
    // No ROI
    std::copy(D, D+5, cam_info_.D.begin());
    std::copy(K, K+9, cam_info_.K.begin());
    std::copy(R, R+9, cam_info_.R.begin());
    std::copy(P, P+12, cam_info_.P.begin());

    model_.fromCameraInfo(cam_info_);
  }
  
  sensor_msgs::CameraInfo cam_info_;
  image_geometry::PinholeCameraModel model_;
};

TEST_F(PinholeTest, accessorsCorrect)
{
  EXPECT_EQ((unsigned)480, model_.height());
  EXPECT_EQ((unsigned)640, model_.width());
  EXPECT_STREQ("tf_frame", model_.tfFrame().c_str());
  EXPECT_EQ(cam_info_.P[0], model_.fx());
  EXPECT_EQ(cam_info_.P[5], model_.fy());
  EXPECT_EQ(cam_info_.P[2], model_.cx());
  EXPECT_EQ(cam_info_.P[6], model_.cy());
}

TEST_F(PinholeTest, projectPoint)
{
  // Spot test an arbitrary point.
  {
    CvPoint2D64f uv = {100, 100};
    CvPoint3D64f xyz;
    model_.projectPixelTo3dRay(uv, xyz);
    EXPECT_DOUBLE_EQ(-0.501371686835080932, xyz.x);
    EXPECT_DOUBLE_EQ(-0.333142973423770972, xyz.y);
    EXPECT_DOUBLE_EQ(0.798525009563579191, xyz.z);
  }

  // Principal point should project straight out.
  {
    CvPoint2D64f uv = {model_.cx(), model_.cy()};
    CvPoint3D64f xyz;
    model_.projectPixelTo3dRay(uv, xyz);
    EXPECT_DOUBLE_EQ(0.0, xyz.x);
    EXPECT_DOUBLE_EQ(0.0, xyz.y);
    EXPECT_DOUBLE_EQ(1.0, xyz.z);
  }
  
  // Check projecting to 3d and back over entire image is accurate.
  const size_t step = 10;
  for (size_t row = 0; row <= cam_info_.height; row += step) {
    for (size_t col = 0; col <= cam_info_.width; col += step) {
      CvPoint2D64f uv = {row, col}, uv_back;
      CvPoint3D64f xyz;
      model_.projectPixelTo3dRay(uv, xyz);
      model_.project3dToPixel(xyz, uv_back);
      // Measured max error at 1.13687e-13
      EXPECT_NEAR(uv.x, uv_back.x, 1.14e-13) << "at (" << row << ", " << col << ")";
      EXPECT_NEAR(uv.y, uv_back.y, 1.14e-13) << "at (" << row << ", " << col << ")";
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
