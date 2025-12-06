from __future__ import print_function

import unittest
import sensor_msgs.msg

from image_geometry import PinholeCameraModel, StereoCameraModel
import numpy as np
from numpy.testing import assert_almost_equal

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
        self.lmsg.d =  [-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05, -0.00044776712298447841, 0.0]
        self.lmsg.k =  [430.15433020105519, 0.0, 311.71339830549732, 0.0, 430.60920415473657, 221.06824942698509, 0.0, 0.0, 1.0]
        self.lmsg.r =  [0.99806560714807102, 0.0068562422224214027, 0.061790256276695904, -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664, -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516]
        self.lmsg.p =  [295.53402059708782, 0.0, 285.55760765075684, 0.0, 0.0, 295.53402059708782, 223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.lmsg.header.frame_id = "left_camera"

        self.rmsg.d =  [-0.3560641041112021, 0.15647260261553159, -0.00016442960757099968, -0.00093175810713916221]
        self.rmsg.k =  [428.38163131344191, 0.0, 327.95553847249192, 0.0, 428.85728580588329, 217.54828640915309, 0.0, 0.0, 1.0]
        self.rmsg.r =  [0.9982082576219119, 0.0067433328293516528, 0.059454199832973849, -0.0068433268864187356, 0.99997549128605434, 0.0014784127772287513, -0.059442773257581252, -0.0018826283666309878, 0.99822993965212292]
        self.rmsg.p =  [295.53402059708782, 0.0, 285.55760765075684, -26.507895206214123, 0.0, 295.53402059708782, 223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rmsg.header.frame_id = "right_camera"

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
        h=480
        w=640
        expected = [12,13,14]
        raw = np.zeros((h, w, 3), np.uint8)
        rectified = np.zeros((h, w, 3), np.uint8)

        for i in range(480):
            for j in range(640):
                for k in range(3):
                    raw[i,j,k] = (i+j+k) % 256         

        self.cam.get_left_camera().rectify_image(raw, rectified)
        assert_almost_equal(expected, rectified[56,47])


    def test_rectify_point(self):
        uv_raw = (1.0, 2.0)
        expected = [48.16447369,45.49210841]
        actual = self.cam.get_left_camera().rectify_point(uv_raw)
        assert_almost_equal(expected,actual,3)

    def test_project_3d_to_pixel(self):
        point = (1.0, 2.0, 3.0)
        expected = [384.069,420.319]
        actual = self.cam.get_left_camera().project_3d_to_pixel(point)
        assert_almost_equal(expected,actual,3)

    def test_project_pixel_to_3d_ray(self):
        uv = (1.0, 2.0)
        expected = [-0.61,-0.475,0.634]
        actual = self.cam.get_left_camera().project_pixel_to_3d_ray(uv)
        assert_almost_equal(expected,actual,3)

    def test_get_delta_u(self):
        delta_x = 1.0
        z = 2.0
        expected = 147.767
        actual = self.cam.get_left_camera().get_delta_u(delta_x,z)
        assert_almost_equal(expected,actual,3)

    def test_get_delta_v(self):
        delta_y = 1.0
        z = 2.0
        expected = 147.767
        actual = self.cam.get_left_camera().get_delta_v(delta_y,z)
        assert_almost_equal(expected,actual,3)

    def test_get_delta_x(self):
        delta_u = 1.0
        z = 2.0
        expected = 0.00676741
        actual = self.cam.get_left_camera().get_delta_x(delta_u,z)
        assert_almost_equal(expected,actual,6)

    def test_get_delta_y(self):
        delta_v = 1.0
        z = 2.0
        expected = 0.00676741
        actual = self.cam.get_left_camera().get_delta_y(delta_v,z)
        assert_almost_equal(expected,actual,6)

    def test_full_resolution(self):
        expected = (640,480)
        actual = self.cam.get_left_camera().full_resolution()
        self.assertTupleEqual(expected,actual)

    def test_intrinsic_matrix(self):
        expected = [[430.15433 ,   0.      , 311.713398],
                    [  0.      , 430.609204, 221.068249],
                    [  0.      ,   0.      ,   1.      ]]
        actual = self.cam.get_left_camera().intrinsic_matrix()
        assert_almost_equal(expected,actual,6)

    def test_distortion_coeffs(self):
        expected = [[-3.63528858e-01],
                    [ 1.61170377e-01],
                    [-8.11095850e-05],
                    [-4.47767123e-04],
                    [ 0.00000000e+00]]
        actual = self.cam.get_left_camera().distortion_coeffs()
        assert_almost_equal(expected,actual,6)

    def test_rotation_matrix(self):
        expected = [[ 0.998066,  0.006856,  0.06179 ],
                    [-0.006752,  0.999975, -0.001891],
                    [-0.061802,  0.00147 ,  0.998087]]
        actual = self.cam.get_left_camera().rotation_matrix()
        assert_almost_equal(expected,actual,6)

    def test_projection_matrix(self):
        expected = [[295.534021,   0.      , 285.557608,   0.      ],
                    [  0.      , 295.534021, 223.296179,   0.      ],
                    [  0.      ,   0.      ,   1.      ,   0.      ]]
        actual = self.cam.get_left_camera().projection_matrix()
        assert_almost_equal(expected,actual,6)

    def test_full_intrinsic_matrix(self):
        expected = [[430.15433 ,   0.      , 311.713398],
                    [  0.      , 430.609204, 221.068249],
                    [  0.      ,   0.      ,   1.      ]]
        actual = self.cam.get_left_camera().full_intrinsic_matrix()
        assert_almost_equal(expected,actual,6)

    def test_full_projection_matrix(self):
        expected = [[295.534021,   0.      , 285.557608,   0.      ],
                    [  0.      , 295.534021, 223.296179,   0.      ],
                    [  0.      ,   0.      ,   1.      ,   0.      ]]
        actual = self.cam.get_left_camera().full_projection_matrix()
        assert_almost_equal(expected,actual,6)

    def test_cx(self):
        expected = 285.557607
        actual = self.cam.get_left_camera().cx()
        assert_almost_equal(expected,actual,6)

    def test_cy(self):
        expected = 223.2961788
        actual = self.cam.get_left_camera().cy()
        assert_almost_equal(expected,actual,6)

    def test_fx(self):
        expected = 295.534020597
        actual = self.cam.get_left_camera().fx()
        assert_almost_equal(expected,actual,6)   

    def test_fy(self):
        expected = 295.534020597
        actual = self.cam.get_left_camera().fy()
        assert_almost_equal(expected,actual,6)   

    def test_tx(self):
        expected = 0.0
        actual = self.cam.get_left_camera().tx()
        assert_almost_equal(expected,actual,6)  

    def test_ty(self):
        expected = 0.0
        actual = self.cam.get_left_camera().ty()
        assert_almost_equal(expected,actual,6)   

    def test_fov_x(self):
        expected = 1.6502496354
        actual = self.cam.get_left_camera().fov_x()
        assert_almost_equal(expected,actual,6)  

    def test_fov_y(self):
        expected = 1.364138172
        actual = self.cam.get_left_camera().fov_y()
        assert_almost_equal(expected,actual,6)  

    def test_get_tf_frame(self):
        expected = "left_camera"
        actual = self.cam.get_left_camera().get_tf_frame()
        self.assertEqual(expected,actual)
            self.assertEqual(expected,actual)
        actual = self.cam.get_tf_frame()
        self.assertEqual(expected,actual)


    def test_stereo_project_3d_to_pixel(self):
        point = (1.0,2.0,3.0)
        expected = (self.cam.get_left_camera().project_3d_to_pixel(point), self.cam.get_right_camera().project_3d_to_pixel(point))
        actual = self.cam.project_3d_to_pixel(point)
        assert_almost_equal(expected, actual, 6)

    def test_stereo_project_pixel_to_3d(self):
        left_uv = (1.0,2.0)
        disparity = 1.1234
        expected = [-22.71975 , -17.668808,  23.596132]
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
        



if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestDirected('test_stereo'))
    suite.addTest(TestDirected('test_rectify_image'))
    suite.addTest(TestDirected('test_rectify_point'))
    suite.addTest(TestDirected('test_project_3d_to_pixel'))
    suite.addTest(TestDirected('test_project_pixel_to_3d_ray'))
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
    suite.addTest(TestDirected('test_get_tf_frame'))
    suite.addTest(TestDirected('test_stereo_project_3d_to_pixel'))
    suite.addTest(TestDirected('test_stereo_project_pixel_to_3d'))
    suite.addTest(TestDirected('test_stereo_get_z'))
    suite.addTest(TestDirected('test_stereo_get_disparity'))
    suite.addTest(TestDirected('test_deprecation'))
    unittest.TextTestRunner(verbosity=3).run(suite)
