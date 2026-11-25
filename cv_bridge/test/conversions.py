import sys
import unittest

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.msg

class TestConversions(unittest.TestCase):

    def test_mono16_cv2(self):
        br = CvBridge()
        im = np.uint8(np.random.randint(0, 255, size=(480, 640, 3)))
        self.assertRaises(CvBridgeError, lambda: br.imgmsg_to_cv2(br.cv2_to_imgmsg(im), 'mono16'))
        br.imgmsg_to_cv2(br.cv2_to_imgmsg(im, 'rgb8'), 'mono16')

    def test_encode_decode_cv2(self):
        import cv2
        fmts = [cv2.CV_8U, cv2.CV_8S, cv2.CV_16U, cv2.CV_16S, cv2.CV_32S, cv2.CV_32F, cv2.CV_64F]

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in fmts:
                    for channels in ([], 1, 2, 3, 4, 5):
                        if channels == []:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w)))
                        else:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w, channels)))
                        rosmsg = cvb_en.cv2_to_imgmsg(original)
                        newimg = cvb_de.imgmsg_to_cv2(rosmsg)

                        self.assertTrue(original.dtype == newimg.dtype)
                        if channels == 1:
                            # in that case, a gray image has a shape of size 2
                            self.assertTrue(original.shape[:2] == newimg.shape[:2])
                        else:
                            self.assertTrue(original.shape == newimg.shape)
                        self.assertTrue(len(original.tobytes()) == len(newimg.tobytes()))

    # From:
    # http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat
    # imread(const string& filename, int flags)
    def test_encode_decode_cv2_compressed(self):
        # FIXME: remove jp2(a.k.a JPEG2000) as its JASPER codec is disabled within Ubuntu opencv library
        # due to security issues, but it works once you rebuild your opencv library with JASPER enabled
        formats = ['jpg', 'jpeg', 'jpe', 'png', 'bmp', 'dib',
                   'sr', 'ras', 'tif', 'tiff']  # this formats rviz is not support

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in formats:
                    for channels in ([], 1, 3):
                        if channels == []:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w)))
                        else:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w, channels)))

                        compress_rosmsg = cvb_en.cv2_to_compressed_imgmsg(original, f)
                        newimg = cvb_de.compressed_imgmsg_to_cv2(compress_rosmsg)
                        self.assertTrue(original.dtype == newimg.dtype)
                        if channels == 1:
                            # in that case, a gray image has a shape of size 2
                            self.assertTrue(original.shape[:2] == newimg.shape[:2])
                        else:
                            self.assertTrue(original.shape == newimg.shape)
                        self.assertTrue(len(original.tobytes()) == len(newimg.tobytes()))

    def test_yuv420_semi_planar(self):
        import cv2
        br = CvBridge()
        rng = np.random.default_rng(0)
        w, h = 64, 48
        for encoding, code in (('nv12', cv2.COLOR_YUV2BGR_NV12), ('nv21', cv2.COLOR_YUV2BGR_NV21)):
            yuv = rng.integers(0, 256, size=(h * 3 // 2, w), dtype=np.uint8)
            msg = br.cv2_to_imgmsg(yuv, encoding)
            self.assertEqual(msg.height, h)
            self.assertEqual(msg.width, w)
            self.assertEqual(msg.step, w)
            self.assertEqual(len(msg.data), w * h * 3 // 2)

            self.assertTrue((br.imgmsg_to_cv2(msg) == yuv).all())
            bgr = br.imgmsg_to_cv2(msg, 'bgr8')
            self.assertEqual(bgr.shape, (h, w, 3))
            self.assertTrue((bgr == cv2.cvtColor(yuv, code)).all())

            self.assertRaises(CvBridgeError,
                              lambda: br.cv2_to_imgmsg(np.zeros((h + 1, w), np.uint8), encoding))

            # Odd width or height is rejected, as in C++
            for odd_w, odd_h in ((w - 1, h), (w, h - 1)):
                odd = br.cv2_to_imgmsg(np.zeros((odd_h, odd_w), np.uint8), 'mono8')
                odd.encoding = encoding
                odd.data = bytes(odd_w * odd_h * 3 // 2 + odd_w)  # large enough for any layout
                self.assertRaises(CvBridgeError, lambda: br.imgmsg_to_cv2(odd))

    def test_endianness(self):
        br = CvBridge()
        dtype = np.dtype('int32')
        # Set to big endian.
        dtype = dtype.newbyteorder('>')
        img = np.random.randint(0, 255, size=(30, 40))
        msg = br.cv2_to_imgmsg(img.astype(dtype))
        self.assertTrue(msg.is_bigendian)
        self.assertTrue((br.imgmsg_to_cv2(msg) == img).all())

    def test_row_padding(self):
        # Rows padded to a 4-byte boundary: the padding is not a whole number of pixels.
        br = CvBridge()
        for encoding, dtype, channels in [('rgb8', np.uint8, 3), ('mono8', np.uint8, 1),
                                          ('16UC3', np.uint16, 3)]:
            h, w = 3, 5
            shape = (h, w, channels) if channels > 1 else (h, w)
            img = np.arange(1, np.prod(shape) + 1).astype(dtype).reshape(shape)
            row_bytes = w * channels * np.dtype(dtype).itemsize
            step = (row_bytes + 3) // 4 * 4
            if step == row_bytes:
                step += 4
            data = np.zeros((h, step), dtype=np.uint8)
            data[:, :row_bytes] = img.view(np.uint8).reshape(h, row_bytes)
            msg = sensor_msgs.msg.Image(height=h, width=w, encoding=encoding, step=step,
                                        is_bigendian=(sys.byteorder == 'big'),
                                        data=data.tobytes())
            np.testing.assert_array_equal(br.imgmsg_to_cv2(msg), img, err_msg=encoding)

    def test_malformed_image(self):
        br = CvBridge()

        def image(step, size):
            return sensor_msgs.msg.Image(height=2, width=4, encoding='rgb8', step=step,
                                         data=bytes(size))

        # A row needs 12 bytes.
        self.assertRaises(CvBridgeError, br.imgmsg_to_cv2, image(step=0, size=24))
        self.assertRaises(CvBridgeError, br.imgmsg_to_cv2, image(step=9, size=24))
        # The second row is not complete.
        self.assertRaises(CvBridgeError, br.imgmsg_to_cv2, image(step=16, size=27))
        # Only the padding of the last row is missing.
        self.assertEqual(br.imgmsg_to_cv2(image(step=16, size=28)).shape, (2, 4, 3))


if __name__ == '__main__':

    suite = unittest.TestSuite()
    suite.addTest(TestConversions('test_mono16_cv2'))
    suite.addTest(TestConversions('test_encode_decode_cv2'))
    suite.addTest(TestConversions('test_encode_decode_cv2_compressed'))
    suite.addTest(TestConversions('test_yuv420_semi_planar'))
    suite.addTest(TestConversions('test_endianness'))
    suite.addTest(TestConversions('test_row_padding'))
    suite.addTest(TestConversions('test_malformed_image'))
    unittest.TextTestRunner(verbosity=2).run(suite)
