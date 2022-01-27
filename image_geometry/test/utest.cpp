#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/distortion_models.hpp>
#include <gtest/gtest.h>

/// @todo Tests with simple values (R = identity, D = 0, P = K or simple scaling)
/// @todo Test projection functions for right stereo values, P(:,3) != 0
/// @todo Tests for rectifyImage
/// @todo Tests using ROI, needs support from PinholeCameraModel
/// @todo Tests for StereoCameraModel

class PinholeTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    /// @todo Just load these from file
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
    cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    // No ROI
    cam_info_.d.resize(5);
    std::copy(D, D+5, cam_info_.d.begin());
    std::copy(K, K+9, cam_info_.k.begin());
    std::copy(R, R+9, cam_info_.r.begin());
    std::copy(P, P+12, cam_info_.p.begin());

    model_.fromCameraInfo(cam_info_);
  }

  sensor_msgs::msg::CameraInfo cam_info_;
  image_geometry::PinholeCameraModel model_;
};

TEST_F(PinholeTest, accessorsCorrect)
{
  EXPECT_STREQ("tf_frame", model_.tfFrame().c_str());
  EXPECT_EQ(cam_info_.p[0], model_.fx());
  EXPECT_EQ(cam_info_.p[5], model_.fy());
  EXPECT_EQ(cam_info_.p[2], model_.cx());
  EXPECT_EQ(cam_info_.p[6], model_.cy());
}

TEST_F(PinholeTest, projectPoint)
{
  // Spot test an arbitrary point.
  {
    cv::Point2d uv(100, 100);
    cv::Point3d xyz =  model_.projectPixelTo3dRay(uv);
    EXPECT_NEAR(-0.62787224048135637, xyz.x, 1e-8);
    EXPECT_NEAR(-0.41719792045817677, xyz.y, 1e-8);
    EXPECT_DOUBLE_EQ(1.0, xyz.z);
  }

  // Principal point should project straight out.
  {
    cv::Point2d uv(model_.cx(), model_.cy());
    cv::Point3d xyz = model_.projectPixelTo3dRay(uv);
    EXPECT_DOUBLE_EQ(0.0, xyz.x);
    EXPECT_DOUBLE_EQ(0.0, xyz.y);
    EXPECT_DOUBLE_EQ(1.0, xyz.z);
  }

  // Check projecting to 3d and back over entire image is accurate.
  const size_t step = 10;
  for (size_t row = 0; row <= cam_info_.height; row += step) {
    for (size_t col = 0; col <= cam_info_.width; col += step) {
      cv::Point2d uv(static_cast<double>(row), static_cast<double>(col)), uv_back;
      cv::Point3d xyz = model_.projectPixelTo3dRay(uv);
      uv_back = model_.project3dToPixel(xyz);
      // Measured max error at 1.13687e-13
      EXPECT_NEAR(uv.x, uv_back.x, 1.14e-13) << "at (" << row << ", " << col << ")";
      EXPECT_NEAR(uv.y, uv_back.y, 1.14e-13) << "at (" << row << ", " << col << ")";
    }
  }
}

TEST_F(PinholeTest, rectifyPoint)
{
  // Spot test an arbitrary point.
  {
    cv::Point2d uv_raw(100, 100), uv_rect;
    uv_rect = model_.rectifyPoint(uv_raw);
    EXPECT_DOUBLE_EQ(142.30311584472656, uv_rect.x);
    EXPECT_DOUBLE_EQ(132.061065673828, uv_rect.y);
  }

  /// @todo Need R = identity for the principal point tests.
#if 0
  // Test rectifyPoint takes (c'x, c'y) [from K] -> (cx, cy) [from P].
  double cxp = model_.intrinsicMatrix()(0,2), cyp = model_.intrinsicMatrix()(1,2);
  {
    cv::Point2d uv_raw(cxp, cyp), uv_rect;
    model_.rectifyPoint(uv_raw, uv_rect);
    EXPECT_NEAR(uv_rect.x, model_.cx(), 1e-4);
    EXPECT_NEAR(uv_rect.y, model_.cy(), 1e-4);
  }

  // Test unrectifyPoint takes (cx, cy) [from P] -> (c'x, c'y) [from K].
  {
    cv::Point2d uv_rect(model_.cx(), model_.cy()), uv_raw;
    model_.unrectifyPoint(uv_rect, uv_raw);
    EXPECT_NEAR(uv_raw.x, cxp, 1e-4);
    EXPECT_NEAR(uv_raw.y, cyp, 1e-4);
  }
#endif

  // Check rectifying then unrectifying over most of the image is accurate.
  const size_t step = 5;
  const size_t border = 65; // Expect bad accuracy far from the center of the image.
  for (size_t row = border; row <= cam_info_.height - border; row += step) {
    for (size_t col = border; col <= cam_info_.width - border; col += step) {
      cv::Point2d uv_raw(static_cast<double>(row), static_cast<double>(col)), uv_rect, uv_unrect;
      uv_rect = model_.rectifyPoint(uv_raw);
      uv_unrect = model_.unrectifyPoint(uv_rect);
      // Check that we're at least within a pixel...
      EXPECT_NEAR(uv_raw.x, uv_unrect.x, 1.0);
      EXPECT_NEAR(uv_raw.y, uv_unrect.y, 1.0);
    }
  }
}

TEST_F(PinholeTest, getDeltas)
{
  double u = 100.0, v = 200.0, du = 17.0, dv = 23.0, Z = 2.0;
  cv::Point2d uv0(u, v), uv1(u + du, v + dv);
  cv::Point3d xyz0, xyz1;
  xyz0 = model_.projectPixelTo3dRay(uv0);
  xyz0 *= (Z / xyz0.z);
  xyz1 = model_.projectPixelTo3dRay(uv1);
  xyz1 *= (Z / xyz1.z);

  EXPECT_NEAR(model_.getDeltaU(xyz1.x - xyz0.x, Z), du, 1e-4);
  EXPECT_NEAR(model_.getDeltaV(xyz1.y - xyz0.y, Z), dv, 1e-4);
  EXPECT_NEAR(model_.getDeltaX(du, Z), xyz1.x - xyz0.x, 1e-4);
  EXPECT_NEAR(model_.getDeltaY(dv, Z), xyz1.y - xyz0.y, 1e-4);
}

TEST_F(PinholeTest, initialization)
{
  EXPECT_EQ(model_.initialized(), true);
  EXPECT_EQ(model_.projectionMatrix().rows, 3);
  EXPECT_EQ(model_.projectionMatrix().cols, 4);
}

TEST_F(PinholeTest, rectifyIfCalibrated)
{
  /// @todo use forward distortion for a better test
  // Ideally this test would have two images stored on disk
  // one which is distorted and the other which is rectified,
  // and then rectification would take place here and the output
  // image compared to the one on disk (which would mean if
  // the distortion coefficients above can't change once paired with
  // an image).

  // Later could incorporate distort code
  // (https://github.com/lucasw/vimjay/blob/master/src/standalone/distort_image.cpp)
  // to take any image distort it, then undistort with rectifyImage,
  // and given the distortion coefficients are consistent the input image
  // and final output image should be mostly the same (though some
  // interpolation error
  // creeps in), except for outside a masked region where information was lost.
  // The masked region can be generated with a pure white image that
  // goes through the same process (if it comes out completely black
  // then the distortion parameters are problematic).

  // For now generate an image and pass the test simply if
  // the rectified image does not match the distorted image.
  // Then zero out the first distortion coefficient and run
  // the test again.
  // Then zero out all the distortion coefficients and test
  // that the output image is the same as the input.
  cv::Mat distorted_image(cv::Size(cam_info_.width, cam_info_.height), CV_8UC3, cv::Scalar(0, 0, 0));

  // draw a grid
  const cv::Scalar color = cv::Scalar(255, 255, 255);
  // draw the lines thick so the proportion of error due to
  // interpolation is reduced
  const int thickness = 7;
  const int type = 8;
  for (size_t y = 0; y <= cam_info_.height; y += cam_info_.height / 10) {
    cv::line(distorted_image,
             cv::Point(0UL, static_cast<uint32_t>(y)),
             cv::Point(static_cast<uint32_t>(cam_info_.width),
                       static_cast<uint32_t>(y)),
             color, type, thickness);
  }
  for (size_t x = 0; x <= cam_info_.width; x += cam_info_.width / 10) {
    // draw the lines thick so the prorportion of interpolation error is reduced
    cv::line(distorted_image,
             cv::Point(static_cast<uint32_t>(x), 0UL),
             cv::Point(static_cast<uint32_t>(x),
                       static_cast<uint32_t>(cam_info_.height)),
             color, type, thickness);
  }

  cv::Mat rectified_image;
  // Just making this number up, maybe ought to be larger
  // since a completely different image would be on the order of
  // width * height * 255 = 78e6
  const double diff_threshold = 10000.0;
  double error;

  // Test that rectified image is sufficiently different
  // using default distortion
  model_.rectifyImage(distorted_image, rectified_image);
  error = cv::norm(distorted_image, rectified_image, cv::NORM_L1);
  // Just making this number up, maybe ought to be larger
  EXPECT_GT(error, diff_threshold);

  // Test that rectified image is sufficiently different
  // using default distortion but with first element zeroed
  // out.
  sensor_msgs::msg::CameraInfo cam_info_2 = cam_info_;
  cam_info_2.d[0] = 0.0;
  model_.fromCameraInfo(cam_info_2);
  model_.rectifyImage(distorted_image, rectified_image);
  error = cv::norm(distorted_image, rectified_image, cv::NORM_L1);
  EXPECT_GT(error, diff_threshold);

  // Test that rectified image is the same using zero distortion
  cam_info_2.d.assign(cam_info_2.d.size(), 0);
  model_.fromCameraInfo(cam_info_2);
  model_.rectifyImage(distorted_image, rectified_image);
  error = cv::norm(distorted_image, rectified_image, cv::NORM_L1);
  EXPECT_EQ(error, 0);

  // Test that rectified image is the same using empty distortion
  cam_info_2.d.clear();
  model_.fromCameraInfo(cam_info_2);
  model_.rectifyImage(distorted_image, rectified_image);
  error = cv::norm(distorted_image, rectified_image, cv::NORM_L1);
  EXPECT_EQ(error, 0);
}

void testUnrectifyImage(const sensor_msgs::msg::CameraInfo& cam_info, const image_geometry::PinholeCameraModel& model)
{
  // test for unrectifyImage: call unrectifyImage, call unrectifyPoint in a loop, compare

  // prepare rectified_image
  cv::Mat rectified_image(model.fullResolution(), CV_8UC3, cv::Scalar(0, 0, 0));

  // draw a grid
  const cv::Scalar color = cv::Scalar(255, 255, 255);
  const int thickness = 7;
  const int type = 8;
  for (int y = 0; y <= rectified_image.rows; y += rectified_image.rows / 10)
  {
    cv::line(rectified_image,
             cv::Point(0, y), cv::Point(cam_info.width, y),
             color, type, thickness);
  }
  for (int x = 0; x <= rectified_image.cols; x += rectified_image.cols / 10)
  {
    cv::line(rectified_image,
             cv::Point(x, 0), cv::Point(x, cam_info.height),
             color, type, thickness);
  }

  // restrict rectified_image to ROI and resize to new binning
  rectified_image = rectified_image(model.rawRoi());
  cv::resize(rectified_image, rectified_image, cv::Size(), 1.0 / model.binningX(), 1.0 / model.binningY(),
             cv::INTER_NEAREST);

  // unrectify image in one go using unrectifyImage
  cv::Mat distorted_image;
  // Just making this number up, maybe ought to be larger
  // since a completely different image would be on the order of
  // width * height * 255 = 78e6
  const double diff_threshold = 10000.0;
  double error;

  // Test that unrectified image is sufficiently different
  // using default distortion
  model.unrectifyImage(rectified_image, distorted_image);
  error = cv::norm(rectified_image, distorted_image, cv::NORM_L1);
  // Just making this number up, maybe ought to be larger
  EXPECT_GT(error, diff_threshold);

  // unrectify image pixel by pixel using unrectifyPoint
  assert(rectified_image.type() == CV_8UC3);  // need this for at<cv::Vec3b> to be correct
  cv::Mat distorted_image_by_pixel = cv::Mat::zeros(rectified_image.size(), rectified_image.type());
  cv::Mat mask = cv::Mat::zeros(rectified_image.size(), CV_8UC1);
  for (int y = 0; y < rectified_image.rows; y++)
  {
    for (int x = 0; x < rectified_image.cols; x++)
    {
      cv::Point2i uv_rect(x, y), uv_raw;

      uv_raw = model.unrectifyPoint(uv_rect);

      if (0 <= uv_raw.x && uv_raw.x < distorted_image_by_pixel.cols && 0 <= uv_raw.y
          && uv_raw.y < distorted_image_by_pixel.rows)
      {
        distorted_image_by_pixel.at<cv::Vec3b>(uv_raw) = rectified_image.at<cv::Vec3b>(uv_rect);
        mask.at<uchar>(uv_raw) = 255;
        // Test that both methods produce similar values at the pixels that unrectifyPoint hits; don't test for all
        // other pixels (the images will differ there, because unrectifyPoint doesn't interpolate missing pixels).
        // Also don't check for absolute equality, but allow a color difference of up to 200. This still catches
        // complete misses (color difference would be 255) while allowing for interpolation at the grid borders.
        EXPECT_LT(distorted_image.at<cv::Vec3b>(uv_raw)[0] - distorted_image_by_pixel.at<cv::Vec3b>(uv_raw)[0], 200);
      }
    }
  }

  // Test that absolute error (due to interpolation) is less than 6% of the maximum possible error
  error = cv::norm(distorted_image, distorted_image_by_pixel, cv::NORM_L1, mask);
  EXPECT_LT(error / (distorted_image.size[0] * distorted_image.size[1] * 255), 0.06);

  // Test that unrectifyPoint hits more than 50% of the output image
  EXPECT_GT((double) cv::countNonZero(mask) / (distorted_image.size[0] * distorted_image.size[1]), 0.5);
};

TEST_F(PinholeTest, unrectifyImage)
{
  testUnrectifyImage(cam_info_, model_);
}

TEST_F(PinholeTest, unrectifyImageWithBinning)
{
  cam_info_.binning_x = 2;
  cam_info_.binning_y = 2;
  model_.fromCameraInfo(cam_info_);

  testUnrectifyImage(cam_info_, model_);
}

TEST_F(PinholeTest, unrectifyImageWithRoi)
{
  cam_info_.roi.x_offset = 100;
  cam_info_.roi.y_offset = 50;
  cam_info_.roi.width = 400;
  cam_info_.roi.height = 300;
  cam_info_.roi.do_rectify = true;
  model_.fromCameraInfo(cam_info_);

  testUnrectifyImage(cam_info_, model_);
}

TEST_F(PinholeTest, unrectifyImageWithBinningAndRoi)
{
  cam_info_.binning_x = 2;
  cam_info_.binning_y = 2;
  cam_info_.roi.x_offset = 100;
  cam_info_.roi.y_offset = 50;
  cam_info_.roi.width = 400;
  cam_info_.roi.height = 300;
  cam_info_.roi.do_rectify = true;
  model_.fromCameraInfo(cam_info_);

  testUnrectifyImage(cam_info_, model_);
}

TEST_F(PinholeTest, rectifiedRoiSize) {

  cv::Rect rectified_roi = model_.rectifiedRoi();
  cv::Size reduced_resolution = model_.reducedResolution();
  EXPECT_EQ(0, rectified_roi.x);
  EXPECT_EQ(0, rectified_roi.y);
  EXPECT_EQ(640, rectified_roi.width);
  EXPECT_EQ(480, rectified_roi.height);
  EXPECT_EQ(640, reduced_resolution.width);
  EXPECT_EQ(480, reduced_resolution.height);

  cam_info_.binning_x = 2;
  cam_info_.binning_y = 2;
  model_.fromCameraInfo(cam_info_);
  rectified_roi = model_.rectifiedRoi();
  reduced_resolution = model_.reducedResolution();
  EXPECT_EQ(0, rectified_roi.x);
  EXPECT_EQ(0, rectified_roi.y);
  EXPECT_EQ(640, rectified_roi.width);
  EXPECT_EQ(480, rectified_roi.height);
  EXPECT_EQ(320, reduced_resolution.width);
  EXPECT_EQ(240, reduced_resolution.height);

  cam_info_.binning_x = 1;
  cam_info_.binning_y = 1;
  cam_info_.roi.x_offset = 100;
  cam_info_.roi.y_offset = 50;
  cam_info_.roi.width = 400;
  cam_info_.roi.height = 300;
  cam_info_.roi.do_rectify = true;
  model_.fromCameraInfo(cam_info_);
  rectified_roi = model_.rectifiedRoi();
  reduced_resolution = model_.reducedResolution();
  EXPECT_EQ(137, rectified_roi.x);
  EXPECT_EQ(82, rectified_roi.y);
  EXPECT_EQ(321, rectified_roi.width);
  EXPECT_EQ(242, rectified_roi.height);
  EXPECT_EQ(321, reduced_resolution.width);
  EXPECT_EQ(242, reduced_resolution.height);

  cam_info_.binning_x = 2;
  cam_info_.binning_y = 2;
  cam_info_.roi.x_offset = 100;
  cam_info_.roi.y_offset = 50;
  cam_info_.roi.width = 400;
  cam_info_.roi.height = 300;
  cam_info_.roi.do_rectify = true;
  model_.fromCameraInfo(cam_info_);
  rectified_roi = model_.rectifiedRoi();
  reduced_resolution = model_.reducedResolution();
  EXPECT_EQ(137, rectified_roi.x);
  EXPECT_EQ(82, rectified_roi.y);
  EXPECT_EQ(321, rectified_roi.width);
  EXPECT_EQ(242, rectified_roi.height);
  EXPECT_EQ(160, reduced_resolution.width);
  EXPECT_EQ(121, reduced_resolution.height);
}

TEST_F(PinholeTest, rectifiedRoiCaching)
{
  // Test that the following sequence works correctly:
  // 1. fromCameraInfo is called with ROI A.  | rectified_roi_dirty = true
  // (already happened in SetUp())

  // 2. rectifiedRoi is called                | rectified_roi_dirty = false
  cv::Rect actual_roi_a = model_.rectifiedRoi();
  cv::Rect expected_roi_a(0, 0, 640, 480);
  EXPECT_EQ(expected_roi_a, actual_roi_a);

  // 3. fromCameraInfo is called with ROI B.  | rectified_roi_dirty = true
  cam_info_.roi.x_offset = 100;
  cam_info_.roi.y_offset = 50;
  cam_info_.roi.width = 400;
  cam_info_.roi.height = 300;
  cam_info_.roi.do_rectify = true;
  model_.fromCameraInfo(cam_info_);

  // 4. fromCameraInfo is called again with ROI B.  | rectified_roi_dirty should still be true!
  model_.fromCameraInfo(cam_info_);

  // 5. rectifiedRoi is called
  // There was a bug before where rectified_roi_dirty was incorrectly set to `false` by step 4.
  // If rectifiedRoi was called again, the cached rectified_roi for
  // ROI A was returned, but it should be recalculated based on ROI B.
  // This test checks that this behavior is correct.
  cv::Rect actual_roi_b = model_.rectifiedRoi();
  cv::Rect expected_roi_b(137, 82, 321, 242);
  EXPECT_EQ(expected_roi_b, actual_roi_b);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
