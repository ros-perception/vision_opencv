from __future__ import print_function

import unittest

from image_geometry import PinholeCameraModel, StereoCameraModel
import numpy as np
from numpy.testing import assert_almost_equal
import sensor_msgs.msg


class TestDirected(unittest.TestCase):

    def setUp(self):
        self.lmsg = sensor_msgs.msg.CameraInfo()
        self.rmsg = sensor_msgs.msg.CameraInfo()
        self.width = 640
        self.height = 480
        for m in (self.lmsg, self.rmsg):
            m.width = self.width
            m.height = self.height

        # These parameters taken from a real camera calibration
        self.lmsg.d = [
            -0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05,
            -0.00044776712298447841, 0.0]
        self.lmsg.k = [
            430.15433020105519, 0.0, 311.71339830549732, 0.0, 430.60920415473657,
            221.06824942698509, 0.0, 0.0, 1.0]
        self.lmsg.r = [
            0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
            -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
            -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516]
        self.lmsg.p = [
            295.53402059708782, 0.0, 285.55760765075684, 0.0, 0.0, 295.53402059708782,
            223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.lmsg.header.frame_id = 'left_camera'

        self.rmsg.d = [
            -0.3560641041112021, 0.15647260261553159, -0.00016442960757099968,
            -0.00093175810713916221]
        self.rmsg.k = [
            428.38163131344191, 0.0, 327.95553847249192, 0.0, 428.85728580588329,
            217.54828640915309, 0.0, 0.0, 1.0]
        self.rmsg.r = [
            0.9982082576219119, 0.0067433328293516528, 0.059454199832973849,
            -0.0068433268864187356, 0.99997549128605434, 0.0014784127772287513,
            -0.059442773257581252, -0.0018826283666309878, 0.99822993965212292]
        self.rmsg.p = [
            295.53402059708782, 0.0, 285.55760765075684, -26.507895206214123, 0.0,
            295.53402059708782, 223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rmsg.header.frame_id = 'right_camera'

        self.cam = StereoCameraModel()
        self.cam.from_camera_info(self.lmsg, self.rmsg)

    # def test_monocular(self):
    #     ci = sensor_msgs.msg.CameraInfo()
    #     ci.width = 640
    #     ci.height = 480
    #     print(ci)
    #     cam = PinholeCameraModel()
    #     cam.from_camera_info(ci)
    #     print(cam.rectify_point((0, 0)))

    #     print(cam.project_3d_to_pixel((0,0,0)))

    def test_stereo(self):
        for x in (16, 320, self.width - 16):
            for y in (16, 240, self.height - 16):
                for d in range(1, 10):
                    pt3d = self.cam.project_pixel_to_3d((x, y), d)
                    ((lx, ly), (rx, ry)) = self.cam.project_3d_to_pixel(pt3d)
                    self.assertAlmostEqual(y, ly, 3)
                    self.assertAlmostEqual(y, ry, 3)
                    self.assertAlmostEqual(x, lx, 3)
                    self.assertAlmostEqual(x, rx + d, 3)

        u = 100.0
        v = 200.0
        du = 17.0
        dv = 23.0
        Z = 2.0
        xyz0 = self.cam.get_left_camera().project_pixel_to_3d_ray((u, v))
        xyz0 = (xyz0[0] * (Z / xyz0[2]), xyz0[1] * (Z / xyz0[2]), Z)
        xyz1 = self.cam.get_left_camera().project_pixel_to_3d_ray((u + du, v + dv))
        xyz1 = (xyz1[0] * (Z / xyz1[2]), xyz1[1] * (Z / xyz1[2]), Z)
        self.assertAlmostEqual(self.cam.get_left_camera().get_delta_u(xyz1[0] - xyz0[0], Z), du, 3)
        self.assertAlmostEqual(self.cam.get_left_camera().get_delta_v(xyz1[1] - xyz0[1], Z), dv, 3)
        self.assertAlmostEqual(self.cam.get_left_camera().get_delta_x(du, Z), xyz1[0] - xyz0[0], 3)
        self.assertAlmostEqual(self.cam.get_left_camera().get_delta_y(dv, Z), xyz1[1] - xyz0[1], 3)

    def test_rectify_image(self):
        h = 480
        w = 640
        expected = [12, 13, 14]
        raw = np.zeros((h, w, 3), np.uint8)
        rectified = np.zeros((h, w, 3), np.uint8)

        for i in range(480):
            for j in range(640):
                for k in range(3):
                    raw[i, j, k] = (i+j+k) % 256

        self.cam.get_left_camera().rectify_image(raw, rectified)
        assert_almost_equal(expected, rectified[56, 47])

    def test_rectify_image_binning_and_roi(self):
        # The raw image of a binned or ROI camera is smaller than the full
        # sensor, so the rectified image must keep that reduced size.
        msg = sensor_msgs.msg.CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.d = [0.0] * 5
        msg.k = [430.0, 0.0, 320.0, 0.0, 430.0, 240.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [430.0, 0.0, 320.0, 0.0, 0.0, 430.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        rng = np.random.default_rng(0)
        for binning, roi in ((2, None), (1, (100, 50, 320, 240)), (2, (100, 50, 320, 240))):
            msg.binning_x = msg.binning_y = binning
            if roi is not None:
                (msg.roi.x_offset, msg.roi.y_offset,
                 msg.roi.width, msg.roi.height) = roi
            cam = PinholeCameraModel()
            cam.from_camera_info(msg)
            width, height = roi[2:] if roi is not None else (self.width, self.height)
            raw = rng.integers(1, 256, (height // binning, width // binning), dtype=np.uint8)
            rectified = np.zeros_like(raw)
            cam.rectify_image(raw, rectified)
            # With no distortion and R = I, rectification leaves the image as it is
            np.testing.assert_array_equal(rectified, raw)

    def test_rectify_image_output_buffer(self):
        # The result is always written to the output image, or an error is raised
        cam = self.cam.get_left_camera()
        rng = np.random.default_rng(0)
        raw = rng.integers(1, 256, (self.height, self.width), dtype=np.uint8)
        expected = np.zeros_like(raw)
        cam.rectify_image(raw, expected)
        self.assertTrue(expected.any())

        # An output image of another type cannot be used by cv2.remap directly
        rectified = np.zeros(raw.shape, dtype=np.float32)
        cam.rectify_image(raw, rectified)
        assert_almost_equal(rectified, expected)

        # An output image of the wrong size is an error, it used to be left untouched
        with self.assertRaises(ValueError):
            cam.rectify_image(raw, np.zeros((self.height // 2, self.width // 2), dtype=np.uint8))

    def test_rectify_point(self):
        uv_raw = (1.0, 2.0)
        expected = [48.16447369, 45.49210841]
        actual = self.cam.get_left_camera().rectify_point(uv_raw)
        assert_almost_equal(expected, actual, 3)

    def test_project_3d_to_pixel(self):
        point = (1.0, 2.0, 3.0)
        expected = [384.069, 420.319]
        actual = self.cam.get_left_camera().project_3d_to_pixel(point)
        assert_almost_equal(expected, actual, 3)

    def test_project_pixel_to_3d_ray(self):
        uv = (1.0, 2.0)
        expected = [-0.61, -0.475, 0.634]
        actual = self.cam.get_left_camera().project_pixel_to_3d_ray(uv)
        assert_almost_equal(expected, actual, 3)

    def test_project_pixel_to_3d_ray_not_normalized(self):
        cam = self.cam.get_left_camera()
        for uv in ((1.0, 2.0), (0.0, 0.0), (self.width - 1.0, self.height - 1.0)):
            unit = cam.project_pixel_to_3d_ray(uv)
            actual = cam.project_pixel_to_3d_ray(uv, normalize=False)
            self.assertEqual(actual[2], 1.0)
            assert_almost_equal([unit[0] / unit[2], unit[1] / unit[2], 1.0], actual, 9)
            # The point on the ray with z = 1 projects back onto the same pixel
            assert_almost_equal(uv, cam.project_3d_to_pixel(actual), 9)
            assert_almost_equal(unit, cam.project_pixel_to_3d_ray(uv, normalize=True), 9)
        # Top-left corner points up and left, bottom-right corner down and right
        assert_almost_equal(
            [-0.966243, -0.755568, 1.0],
            cam.project_pixel_to_3d_ray((0.0, 0.0), normalize=False), 6)
        assert_almost_equal(
            [1.195945, 0.865226, 1.0],
            cam.project_pixel_to_3d_ray((self.width - 1.0, self.height - 1.0), normalize=False),
            6)

    def test_project_pixel_to_3d_ray_principal_point(self):
        cam = self.cam.get_left_camera()
        uv = (cam.cx(), cam.cy())
        self.assertEqual(cam.project_pixel_to_3d_ray(uv, normalize=False), (0.0, 0.0, 1.0))
        assert_almost_equal([0.0, 0.0, 1.0], cam.project_pixel_to_3d_ray(uv), 12)

    def test_project_pixel_to_3d_ray_not_normalized_right_camera(self):
        # The right camera has Tx != 0. The ray starts at its own center, so Tx is not
        # subtracted (unlike the C++ projectPixelTo3dRay()).
        left = self.cam.get_left_camera()
        right = self.cam.get_right_camera()
        self.assertNotEqual(right.tx(), 0.0)
        uv = (1.0, 2.0)
        unit = right.project_pixel_to_3d_ray(uv)
        actual = right.project_pixel_to_3d_ray(uv, normalize=False)
        self.assertEqual(actual[2], 1.0)
        assert_almost_equal([unit[0] / unit[2], unit[1] / unit[2], 1.0], actual, 9)
        # Both cameras share fx, fy, cx and cy, so the directions are the same
        assert_almost_equal(left.project_pixel_to_3d_ray(uv, normalize=False), actual, 9)
        # Offset by the right camera's center (-Tx / fx, 0, 0), the point projects back onto uv
        center_x = -right.tx() / right.fx()
        assert_almost_equal(
            uv, right.project_3d_to_pixel((center_x + actual[0], actual[1], actual[2])), 9)

    def test_project_pixel_to_3d_ray_not_normalized_fx_fy(self):
        # Different focal lengths and principal point coordinates, so swapping them is caught
        msg = sensor_msgs.msg.CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.k = [400.0, 0.0, 300.0, 0.0, 200.0, 250.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [400.0, 0.0, 300.0, 0.0, 0.0, 200.0, 250.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        cam = PinholeCameraModel()
        cam.from_camera_info(msg)
        actual = cam.project_pixel_to_3d_ray((500.0, 150.0), normalize=False)
        assert_almost_equal([0.5, -0.5, 1.0], actual, 12)
        assert_almost_equal((500.0, 150.0), cam.project_3d_to_pixel(actual), 9)

    def test_project_pixel_to_3d_ray_normalize_is_keyword_only(self):
        with self.assertRaises(TypeError):
            self.cam.get_left_camera().project_pixel_to_3d_ray((1.0, 2.0), False)

    def test_get_delta_u(self):
        delta_x = 1.0
        z = 2.0
        expected = 147.767
        actual = self.cam.get_left_camera().get_delta_u(delta_x, z)
        assert_almost_equal(expected, actual, 3)

    def test_get_delta_v(self):
        delta_y = 1.0
        z = 2.0
        expected = 147.767
        actual = self.cam.get_left_camera().get_delta_v(delta_y, z)
        assert_almost_equal(expected, actual, 3)

    def test_get_delta_x(self):
        delta_u = 1.0
        z = 2.0
        expected = 0.00676741
        actual = self.cam.get_left_camera().get_delta_x(delta_u, z)
        assert_almost_equal(expected, actual, 6)

    def test_get_delta_y(self):
        delta_v = 1.0
        z = 2.0
        expected = 0.00676741
        actual = self.cam.get_left_camera().get_delta_y(delta_v, z)
        assert_almost_equal(expected, actual, 6)

    def test_full_resolution(self):
        expected = (640, 480)
        actual = self.cam.get_left_camera().full_resolution()
        self.assertTupleEqual(expected, actual)

    def test_intrinsic_matrix(self):
        expected = [[430.15433, 0., 311.713398],
                    [0., 430.609204, 221.068249],
                    [0., 0., 1.]]
        actual = self.cam.get_left_camera().intrinsic_matrix()
        assert_almost_equal(expected, actual, 6)

    def test_distortion_coeffs(self):
        expected = [[-3.63528858e-01],
                    [1.61170377e-01],
                    [-8.11095850e-05],
                    [-4.47767123e-04],
                    [0.00000000e+00]]
        actual = self.cam.get_left_camera().distortion_coeffs()
        assert_almost_equal(expected, actual, 6)

    def test_rotation_matrix(self):
        expected = [[0.998066, 0.006856, 0.06179],
                    [-0.006752, 0.999975, -0.001891],
                    [-0.061802, 0.00147, 0.998087]]
        actual = self.cam.get_left_camera().rotation_matrix()
        assert_almost_equal(expected, actual, 6)

    def test_projection_matrix(self):
        expected = [[295.534021, 0., 285.557608, 0.],
                    [0., 295.534021, 223.296179, 0.],
                    [0., 0., 1., 0.]]
        actual = self.cam.get_left_camera().projection_matrix()
        assert_almost_equal(expected, actual, 6)

    def test_full_intrinsic_matrix(self):
        expected = [[430.15433, 0., 311.713398],
                    [0., 430.609204, 221.068249],
                    [0., 0., 1.]]
        actual = self.cam.get_left_camera().full_intrinsic_matrix()
        assert_almost_equal(expected, actual, 6)

    def test_full_projection_matrix(self):
        expected = [[295.534021, 0., 285.557608, 0.],
                    [0., 295.534021, 223.296179, 0.],
                    [0., 0., 1., 0.]]
        actual = self.cam.get_left_camera().full_projection_matrix()
        assert_almost_equal(expected, actual, 6)

    def test_cx(self):
        expected = 285.557607
        actual = self.cam.get_left_camera().cx()
        assert_almost_equal(expected, actual, 6)

    def test_cy(self):
        expected = 223.2961788
        actual = self.cam.get_left_camera().cy()
        assert_almost_equal(expected, actual, 6)

    def test_fx(self):
        expected = 295.534020597
        actual = self.cam.get_left_camera().fx()
        assert_almost_equal(expected, actual, 6)

    def test_fy(self):
        expected = 295.534020597
        actual = self.cam.get_left_camera().fy()
        assert_almost_equal(expected, actual, 6)

    def test_tx(self):
        expected = 0.0
        actual = self.cam.get_left_camera().tx()
        assert_almost_equal(expected, actual, 6)

    def test_ty(self):
        expected = 0.0
        actual = self.cam.get_left_camera().ty()
        assert_almost_equal(expected, actual, 6)

    def test_fov_x(self):
        expected = 1.6502496354
        actual = self.cam.get_left_camera().fov_x()
        assert_almost_equal(expected, actual, 6)

    def test_fov_y(self):
        expected = 1.364138172
        actual = self.cam.get_left_camera().fov_y()
        assert_almost_equal(expected, actual, 6)

    def test_fov_with_binning(self):
        # Binning lowers the image resolution, not the field of view.
        self.lmsg.binning_x = 2
        self.lmsg.binning_y = 4
        binned = PinholeCameraModel()
        binned.from_camera_info(self.lmsg)
        assert_almost_equal(1.6502496354, binned.fov_x(), 6)
        assert_almost_equal(1.364138172, binned.fov_y(), 6)

    def test_get_tf_frame(self):
        expected = 'left_camera'
        actual = self.cam.get_left_camera().get_tf_frame()
        self.assertEqual(expected, actual)
        actual = self.cam.get_tf_frame()
        self.assertEqual(expected, actual)

    def test_stamp(self):
        self.lmsg.header.stamp.sec = 12
        self.lmsg.header.stamp.nanosec = 34
        self.cam.from_camera_info(self.lmsg, self.rmsg)
        actual = self.cam.get_left_camera().stamp()
        self.assertEqual(self.lmsg.header.stamp, actual)

    def test_raw_roi(self):
        # A region of interest that is all zeros is the full resolution
        actual = self.cam.get_left_camera().raw_roi()
        self.assertEqual((0, 0, 640, 480),
                         (actual.x_offset, actual.y_offset, actual.width, actual.height))

        self.lmsg.roi.x_offset = 10
        self.lmsg.roi.y_offset = 20
        self.lmsg.roi.width = 100
        self.lmsg.roi.height = 50
        self.cam.from_camera_info(self.lmsg, self.rmsg)
        actual = self.cam.get_left_camera().raw_roi()
        self.assertEqual((10, 20, 100, 50),
                         (actual.x_offset, actual.y_offset, actual.width, actual.height))

    def test_binning(self):
        # No binning in the message is a binning of 1
        self.assertEqual(1, self.cam.get_left_camera().binning_x())
        self.assertEqual(1, self.cam.get_left_camera().binning_y())

        self.lmsg.binning_x = 2
        self.lmsg.binning_y = 4
        self.cam.from_camera_info(self.lmsg, self.rmsg)
        self.assertEqual(2, self.cam.get_left_camera().binning_x())
        self.assertEqual(4, self.cam.get_left_camera().binning_y())

    def test_stereo_reprojection_matrix(self):
        q = self.cam.reprojection_matrix()
        self.assertEqual((4, 4), q.shape)
        # Reprojecting a pixel and its disparity gives the same point as project_pixel_to_3d
        left_uv = (1.0, 2.0)
        disparity = 1.1234
        point = q @ np.array([left_uv[0], left_uv[1], disparity, 1.0])
        assert_almost_equal(
            self.cam.project_pixel_to_3d(left_uv, disparity), point[:3] / point[3], 6)

    def test_stereo_project_3d_to_pixel(self):
        point = (1.0, 2.0, 3.0)
        expected = (self.cam.get_left_camera().project_3d_to_pixel(point),
                    self.cam.get_right_camera().project_3d_to_pixel(point))
        actual = self.cam.project_3d_to_pixel(point)
        assert_almost_equal(expected, actual, 6)

    def test_stereo_project_pixel_to_3d(self):
        left_uv = (1.0, 2.0)
        disparity = 1.1234
        expected = [-22.71975, -17.668808, 23.596132]
        actual = self.cam.project_pixel_to_3d(left_uv, disparity)
        assert_almost_equal(expected, actual, 6)

    def test_stereo_get_z(self):
        disparity = 1.1234
        expected = 23.59613246
        actual = self.cam.get_z(disparity)
        assert_almost_equal(expected, actual, 6)

    def test_stereo_get_disparity(self):
        z = 23.59613246
        expected = 1.1234
        actual = self.cam.get_disparity(z)
        assert_almost_equal(expected, actual, 6)

    def test_binned_tx_ty(self):
        # Tx and Ty in P are in full-resolution pixels, so binning has to
        # scale them like fx and fy, or a binned right camera projects to the
        # wrong pixel and the stereo depth is off by the binning factor.
        def stereo(binning, ty=0.0):
            lmsg = sensor_msgs.msg.CameraInfo()
            rmsg = sensor_msgs.msg.CameraInfo()
            for m, tx in ((lmsg, 0.0), (rmsg, -50.0)):
                m.width = self.width
                m.height = self.height
                m.binning_x = m.binning_y = binning
                m.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
                m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                m.p = [500.0, 0.0, 320.0, tx, 0.0, 500.0, 240.0, ty, 0.0, 0.0, 1.0, 0.0]
            cam = StereoCameraModel()
            cam.from_camera_info(lmsg, rmsg)
            return cam

        full = stereo(1)
        binned = stereo(2)
        point = (0.2, 0.1, 2.0)
        ((lu, lv), (ru, rv)) = full.project_3d_to_pixel(point)
        expected = ((lu / 2, lv / 2), (ru / 2, rv / 2))
        assert_almost_equal(binned.project_3d_to_pixel(point), expected, 6)
        assert_almost_equal(binned.get_right_camera().tx(), -25.0, 6)

        disparity = (lu - ru) / 2
        assert_almost_equal(binned.get_z(disparity), point[2], 6)
        assert_almost_equal(binned.get_disparity(point[2]), disparity, 6)
        assert_almost_equal(binned.project_pixel_to_3d(expected[0], disparity), point, 6)

        # Ty scales with binning_y in the same way
        full_ty = stereo(1, ty=-30.0).get_right_camera().project_3d_to_pixel(point)
        binned_ty = stereo(2, ty=-30.0).get_right_camera().project_3d_to_pixel(point)
        assert_almost_equal(binned_ty, (full_ty[0] / 2, full_ty[1] / 2), 6)

    def test_stereo_principal_point_offset(self):
        # Rectifying without CALIB_ZERO_DISPARITY can leave cx different in
        # the two cameras. The disparity x_left - x_right then includes the
        # offset cx_left - cx_right, so Q and get_z() have to remove it.
        lmsg = sensor_msgs.msg.CameraInfo()
        rmsg = sensor_msgs.msg.CameraInfo()
        for m, cx, tx in ((lmsg, 320.0, 0.0), (rmsg, 340.0, -50.0)):
            m.width = self.width
            m.height = self.height
            m.k = [500.0, 0.0, cx, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
            m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            m.p = [500.0, 0.0, cx, tx, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        cam = StereoCameraModel()
        cam.from_camera_info(lmsg, rmsg)

        # u_left = 500 * 0.2 / 2 + 320, u_right = 500 * 0.2 / 2 + 340 - 50 / 2
        point = (0.2, 0.1, 2.0)
        ((lu, lv), (ru, rv)) = cam.project_3d_to_pixel(point)
        assert_almost_equal(((lu, lv), (ru, rv)), ((370.0, 265.0), (365.0, 265.0)), 6)
        disparity = lu - ru

        assert_almost_equal(cam.project_pixel_to_3d((lu, lv), disparity), point, 6)
        assert_almost_equal(cam.get_z(disparity), point[2], 6)
        assert_almost_equal(cam.get_disparity(point[2]), disparity, 6)
        self.assertEqual(float('inf'), cam.get_z(-20.0))

        # Baseline 0.1: Q[3, 3] = (cx_right - cx_left) / 0.1
        expected_q = [[1.0, 0.0, 0.0, -320.0],
                      [0.0, 1.0, 0.0, -240.0],
                      [0.0, 0.0, 0.0, 500.0],
                      [0.0, 0.0, 10.0, 200.0]]
        assert_almost_equal(cam.reprojection_matrix(), expected_q, 6)


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestDirected('test_stereo'))
    suite.addTest(TestDirected('test_rectify_image'))
    suite.addTest(TestDirected('test_rectify_image_binning_and_roi'))
    suite.addTest(TestDirected('test_rectify_image_output_buffer'))
    suite.addTest(TestDirected('test_rectify_point'))
    suite.addTest(TestDirected('test_project_3d_to_pixel'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray_not_normalized'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray_principal_point'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray_not_normalized_right_camera'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray_not_normalized_fx_fy'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray_normalize_is_keyword_only'))
    suite.addTest(TestDirected('test_get_delta_u'))
    suite.addTest(TestDirected('test_get_delta_v'))
    suite.addTest(TestDirected('test_get_delta_x'))
    suite.addTest(TestDirected('test_get_delta_y'))
    suite.addTest(TestDirected('test_full_resolution'))
    suite.addTest(TestDirected('test_intrinsic_matrix'))
    suite.addTest(TestDirected('test_distortion_coeffs'))
    suite.addTest(TestDirected('test_rotation_matrix'))
    suite.addTest(TestDirected('test_projection_matrix'))
    suite.addTest(TestDirected('test_full_intrinsic_matrix'))
    suite.addTest(TestDirected('test_full_projection_matrix'))
    suite.addTest(TestDirected('test_cx'))
    suite.addTest(TestDirected('test_cy'))
    suite.addTest(TestDirected('test_fx'))
    suite.addTest(TestDirected('test_fy'))
    suite.addTest(TestDirected('test_tx'))
    suite.addTest(TestDirected('test_ty'))
    suite.addTest(TestDirected('test_fov_x'))
    suite.addTest(TestDirected('test_fov_y'))
    suite.addTest(TestDirected('test_fov_with_binning'))
    suite.addTest(TestDirected('test_get_tf_frame'))
    suite.addTest(TestDirected('test_stereo_project_3d_to_pixel'))
    suite.addTest(TestDirected('test_stereo_project_pixel_to_3d'))
    suite.addTest(TestDirected('test_stereo_get_z'))
    suite.addTest(TestDirected('test_stereo_get_disparity'))
    suite.addTest(TestDirected('test_binned_tx_ty'))
    suite.addTest(TestDirected('test_stereo_principal_point_offset'))
    suite.addTest(TestDirected('test_deprecation'))
    unittest.TextTestRunner(verbosity=3).run(suite)
