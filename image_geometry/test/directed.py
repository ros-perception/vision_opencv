import roslib
PKG = 'image_geometry'
roslib.load_manifest(PKG)
import rostest
import rospy
import cv
import unittest
import sensor_msgs.msg

from image_geometry import PinholeCameraModel, StereoCameraModel

class TestDirected(unittest.TestCase):

    def setUp(self):
        pass

    def test_monocular(self):
        ci = sensor_msgs.msg.CameraInfo()
        ci.width = 640
        ci.height = 480
        print ci
        cam = PinholeCameraModel()
        cam.fromCameraInfo(ci)
        print cam.rectifyPoint((0, 0))

    def test_stereo(self):
        pass

if __name__ == '__main__':
    if 0:
        rostest.unitrun('image_geometry', 'directed', TestDirected)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_monocular'))
        unittest.TextTestRunner(verbosity=2).run(suite)
