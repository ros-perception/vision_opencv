#!/usr/bin/env python
import rostest
import unittest

import sensor_msgs.msg

from cv_bridge import CvBridge, CvBridgeError

class TestEnumerants(unittest.TestCase):

    def test_enumerants_cv(self):
        import cv2

        img_msg = sensor_msgs.msg.Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.encoding = "rgba8"
        img_msg.step = 640*4
        img_msg.data = (640 * 480) * "1234"

        bridge_ = CvBridge()
        cvim = bridge_.imgmsg_to_cv2(img_msg, "rgb8")

        # A 3 channel image cannot be sent as an rgba8
        self.assertRaises(CvBridgeError, lambda: bridge_.cv2_to_imgmsg(cvim, "rgba8"))

        # but it can be sent as rgb8 and bgr8
        bridge_.cv2_to_imgmsg(cvim, "rgb8")
        bridge_.cv2_to_imgmsg(cvim, "bgr8")

    def test_enumerants_cv2(self):
        img_msg = sensor_msgs.msg.Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.encoding = "rgba8"
        img_msg.step = 640*4
        img_msg.data = (640 * 480) * "1234"

        bridge_ = CvBridge()
        cvim = bridge_.imgmsg_to_cv2(img_msg, "rgb8")
        import sys
        self.assertRaises(sys.getrefcount(cvim) == 2)

        # A 3 channel image cannot be sent as an rgba8
        self.assertRaises(CvBridgeError, lambda: bridge_.cv2_to_imgmsg(cvim, "rgba8"))

        # but it can be sent as rgb8 and bgr8
        bridge_.cv2_to_imgmsg(cvim, "rgb8")
        bridge_.cv2_to_imgmsg(cvim, "bgr8")

    def test_encode_decode_cv2(self):
        import cv2
        import numpy as np
        fmts = [ cv2.CV_8U, cv2.CV_8S, cv2.CV_16U, cv2.CV_16S, cv2.CV_32S, cv2.CV_32F, cv2.CV_64F ]

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in fmts:
                    for channels in (1,2,3,4):
                        original = np.uint8(np.random.randint(0, 255, size=(h, w, channels)))
                        rosmsg = cvb_en.cv2_to_imgmsg(original)
                        newimg = cvb_de.imgmsg_to_cv2(rosmsg)
                        self.assert_(original.dtype == newimg.dtype)
                        self.assert_(original.shape == newimg.shape)
                        self.assert_(len(original.tostring()) == len(newimg.tostring()))

    def test_mono16_cv2(self):
        import numpy as np
        br = CvBridge()
        im = np.uint8(np.random.randint(0, 255, size=(480, 640, 3)))
        msg = br.cv2_to_imgmsg(im)

if __name__ == '__main__':
    rostest.unitrun('opencv_tests', 'enumerants', TestEnumerants)
