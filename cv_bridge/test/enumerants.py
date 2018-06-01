import unittest

import cv2
from cv_bridge import CvBridge, CvBridgeError, getCvType
import sensor_msgs.msg


class TestEnumerants(unittest.TestCase):

    def test_enumerants_cv2(self):
        img_msg = sensor_msgs.msg.Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.encoding = 'rgba8'
        img_msg.step = 640 * 4
        img_msg.data = ((640 * 480) * '1234').encode()

        bridge_ = CvBridge()
        cvim = bridge_.imgmsg_to_cv2(img_msg, 'rgb8')

        import sys
        self.assertTrue(sys.getrefcount(cvim) == 2)

        # A 3 channel image cannot be sent as an rgba8
        self.assertRaises(CvBridgeError, lambda: bridge_.cv2_to_imgmsg(cvim, 'rgba8'))

        # but it can be sent as rgb8 and bgr8
        bridge_.cv2_to_imgmsg(cvim, 'rgb8')
        bridge_.cv2_to_imgmsg(cvim, 'bgr8')

        self.assertFalse(getCvType('32FC4') == cv2.CV_8UC4)
        self.assertTrue(getCvType('8UC1') == cv2.CV_8UC1)
        self.assertTrue(getCvType('8U') == cv2.CV_8UC1)

    def test_numpy_types(self):
        bridge_ = CvBridge()
        self.assertRaises(TypeError, lambda: bridge_.cv2_to_imgmsg(1, 'rgba8'))
        if hasattr(cv2, 'cv'):
            self.assertRaises(TypeError, lambda: bridge_.cv2_to_imgmsg(cv2.cv(), 'rgba8'))


if __name__ == '__main__':

    suite = unittest.TestSuite()
    suite.addTest(TestEnumerants('test_enumerants_cv2'))
    suite.addTest(TestEnumerants('test_numpy_types'))
    unittest.TextTestRunner(verbosity=2).run(suite)
