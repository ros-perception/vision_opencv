import roslib; roslib.load_manifest('cv_markers')

import cv
import numpy

import rostest
import rospy
import unittest

class TestSmoke(unittest.TestCase):

    def test_smoke(self):

        im = cv.CreateMat(640, 480, cv.CV_8UC1)
        self.assertEqual(cv.FindDataMatrix(im), [])

if __name__ == '__main__':
  rostest.unitrun('cv_markers_test', 'smoke', TestSmoke)
