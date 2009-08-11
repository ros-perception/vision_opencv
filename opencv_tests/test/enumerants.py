import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

from opencv_latest.cv_bridge import CvBridge, CvBridgeError

class TestEnumerants(unittest.TestCase):

    def test_enumerants(self):

        img_msg = sensor_msgs.msg.Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.encoding = "rgba8"
        img_msg.step = 640*4
        img_msg.data = (640 * 480) * "1234"

        bridge_ = CvBridge()
        cvim = bridge_.imgmsg_to_cv(img_msg, "rgb8")

        # A 3 channel image cannot be sent as an rgba8
        self.assertRaises(CvBridgeError, lambda: bridge_.cv_to_imgmsg(cvim, "rgba8"))

        # but it can be sent as rgb8 and bgr8
        bridge_.cv_to_imgmsg(cvim, "rgb8")
        bridge_.cv_to_imgmsg(cvim, "bgr8")

    def test_encode_decode(self):
        fmts = [ cv.IPL_DEPTH_8U, cv.IPL_DEPTH_8S, cv.IPL_DEPTH_16U, cv.IPL_DEPTH_16S, cv.IPL_DEPTH_32S, cv.IPL_DEPTH_32F, cv.IPL_DEPTH_64F ]

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in  [ cv.IPL_DEPTH_8U ]:
                    for channels in (1,2,3,4):
                        original = cv.CreateImage((w, h), f, channels)
                        cv.Set(original, (1,2,3,4));
                        rosmsg = cvb_en.cv_to_imgmsg(original)
                        newimg = cvb_de.imgmsg_to_cv(rosmsg)
                        self.assert_(cv.GetElemType(original) == cv.GetElemType(newimg))
                        self.assert_(cv.GetSize(original) == cv.GetSize(newimg))
                        self.assert_(len(original.tostring()) == len(newimg.tostring()))

if __name__ == '__main__':
  if 0:
    rostest.unitrun('opencv_tests', 'enumerants', TestEnumerants)
  else:
    suite = unittest.TestSuite()
    suite.addTest(TestEnumerants('test_encode_decode'))
    unittest.TextTestRunner(verbosity=2).run(suite)
