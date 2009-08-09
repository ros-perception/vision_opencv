import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

class TestEnumerants(unittest.TestCase):

    def test_enumerants(self):
        pass

if __name__ == '__main__':
  if 1:
    rostest.unitrun('opencv_tests', 'enumerants', TestEnumerants)
  else:
    suite = unittest.TestSuite()
    suite.addTest(TestEnumerants('test_enumerants'))
    unittest.TextTestRunner(verbosity=2).run(suite)
