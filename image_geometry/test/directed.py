import roslib
PKG = 'image_geometry'
roslib.load_manifest(PKG)
import rostest
import rospy
import cv
import unittest

class TestDirected(unittest.TestCase):

    def setUp(self):
        pass

    def test_monocular(self):
        pass

    def test_stereo(self):
        pass

if __name__ == '__main__':
    if 0:
        rostest.unitrun('camera_calibration', 'directed', TestDirected)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_monocular'))
        unittest.TextTestRunner(verbosity=2).run(suite)
