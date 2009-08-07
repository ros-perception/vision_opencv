import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

class TestEnumerants(unittest.TestCase):

    def test_enumerants(self):
        for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F" ]:
          for c in [1,2,3,4]:
            nm = "%sC%d" % (t, c)
            cv_num = eval("cv.CV_%s" % nm)
            msg_num = eval("sensor_msgs.msg.Image.TYPE_%s" % nm)
            self.assertEqual(cv_num, msg_num)

if __name__ == '__main__':
  if 1:
    rostest.unitrun('opencv_tests', 'enumerants', TestEnumerants)
  else:
    suite = unittest.TestSuite()
    suite.addTest(TestEnumerants('test_enumerants'))
    unittest.TextTestRunner(verbosity=2).run(suite)
