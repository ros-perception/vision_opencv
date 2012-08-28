import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

class CvBridgeError(TypeError):
    """
    This is the error raised by :class:`opencv_latest.cv_bridge.CvBridge` methods when they fail.
    """
    pass

class CvBridge:
    """
    The CvBridge is an object that converts between OpenCV Images and ROS Image messages.

       .. doctest::
           :options: -ELLIPSIS, +NORMALIZE_WHITESPACE

           >>> import cv
           >>> from cv_bridge import CvBridge
           >>> im = cv.CreateImage((640, 480), 8, 3)
           >>> br = CvBridge()
           >>> msg = br.cv_to_imgmsg(im)  # Convert the image to a message
           >>> im2 = br.imgmsg_to_cv(msg) # Convert the message to a new image
           >>> cv.SaveImage("this_was_a_message_briefly.png", im2)

    """

    def __init__(self):
        self.cvtype_names = {}

        for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F" ]:
            for c in [1,2,3,4]:
                nm = "%sC%d" % (t, c)
                self.cvtype_names[eval("cv.CV_%s" % nm)] = nm

    def encoding_as_cvtype(self, encoding):
        channeltypes = {
          "rgb8" : cv.CV_8UC3,
          "bgr8" : cv.CV_8UC3,
          "rgba8" : cv.CV_8UC4,
          "bgra8" : cv.CV_8UC4,
          "mono8" : cv.CV_8UC1,
          "mono16" : cv.CV_16UC1
        }

        if encoding in channeltypes:
            return channeltypes[encoding]
        else:
            try:
                return eval("cv.CV_%s" % encoding)
            except AttributeError:
                print 'Unknown OpenCV format %s' % encoding
                raise

    def encoding_as_fmt(self, encoding):
        source_channels = cv.CV_MAT_CN(self.encoding_as_cvtype(encoding))
        if source_channels == 1:
          fmt = "GRAY"
        elif "rgb8" == encoding:
          fmt = "RGB"
        elif "rgba8" == encoding:
          fmt = "RGBA"
        elif source_channels == 3:
          fmt = "BGR"
        elif source_channels == 4:
          fmt = "BGRA"
        return fmt

    def imgmsg_to_cv(self, img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::Image message to an OpenCV :ctype:`IplImage`.

        :param img_msg:   A sensor_msgs::Image message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * ``"rgb8"``
           * ``"rgba8"``
           * ``"bgr8"``
           * ``"bgra8"``
           * ``"mono8"``
           * ``"mono16"``

        :rtype: :ctype:`IplImage`
        :raises CvBridgeError: when conversion is not possible.
            
        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the strings "rgb8", "bgr8", "rgba8", "bgra8", "mono8" or "mono16",
        in which case this method converts the image using
        :func:`CvtColor`
        (if necessary) and the returned image has a type as follows:

           ``CV_8UC3``
                for "rgb8", "bgr8"
           ``CV_8UC4``
                for "rgba8", "bgra8"
           ``CV_8UC1``
                for "mono8"
           ``CV_16UC1``
                for "mono16"

        This function returns an OpenCV :ctype:`IplImage` message on success, or raises :exc:`opencv_latest.cv_bridge.CvBridgeError` on failure.
        """

        source_type = self.encoding_as_cvtype(img_msg.encoding)
        im = cv.CreateMatHeader(img_msg.height, img_msg.width, source_type)
        cv.SetData(im, img_msg.data, img_msg.step)

        if desired_encoding == "passthrough":
            return im

        # Might need to do a conversion.  sourcefmt and destfmt can be
        # one of GRAY, RGB, BGR, RGBA, BGRA.
        sourcefmt = self.encoding_as_fmt(img_msg.encoding)
        destfmt = self.encoding_as_fmt(desired_encoding)

        source_type = self.encoding_as_cvtype(img_msg.encoding)
        destination_type = self.encoding_as_cvtype(desired_encoding)
        if sourcefmt == destfmt and source_type == destination_type:
            return im

        # First want to make sure that source depth matches destination depth
        if source_type != destination_type:
            # im2 is the intermediate image. It has the same # channels as source_type,
            # but the depth of destination_type.

            # XXX - these macros were missing from OpenCV Python, so roll our own here:
            CV_CN_SHIFT = 3
            def CV_MAKETYPE(depth,cn):
                return cv.CV_MAT_DEPTH(depth) + ((cn - 1) << CV_CN_SHIFT)

            im2_type = CV_MAKETYPE(destination_type, cv.CV_MAT_CN(source_type))
            im2 = cv.CreateMat(img_msg.height, img_msg.width, im2_type)
            cv.ConvertScale(im, im2)
        else:
            im2 = im

        if sourcefmt != destfmt:
            im3 = cv.CreateMat(img_msg.height, img_msg.width, destination_type)
            cv.CvtColor(im2, im3, eval("cv.CV_%s2%s" % (sourcefmt, destfmt)))
        else:
            im3 = im2
        return im3

    def cv_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :ctype:`CvArr` type (that is, an :ctype:`IplImage` or :ctype:`CvMat`) to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :ctype:`IplImage` or :ctype:`CvMat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * ``"rgb8"``
           * ``"rgba8"``
           * ``"bgr8"``
           * ``"bgra8"``
           * ``"mono8"``
           * ``"mono16"``

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
            
        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise encoding must be one of the strings "rgb8", "bgr8", "rgba8", "bgra8", "mono8" or "mono16",
        in which case the OpenCV image must have the appropriate type:

           ``CV_8UC3``
                for "rgb8", "bgr8"
           ``CV_8UC4``
                for "rgba8", "bgra8"
           ``CV_8UC1``
                for "mono8"
           ``CV_16UC1``
                for "mono16"

        This function returns a sensor_msgs::Image message on success, or raises :exc:`opencv_latest.cv_bridge.CvBridgeError` on failure.
        """
        img_msg = sensor_msgs.msg.Image()
        (img_msg.width, img_msg.height) = cv.GetSize(cvim)
        if encoding == "passthrough":
            img_msg.encoding = self.cvtype_names[cv.GetElemType(cvim)]
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.encoding_as_cvtype(encoding) != cv.GetElemType(cvim):
              raise CvBridgeError, "encoding specified as %s, but image has incompatible type %s" % (encoding, self.cvtype_names[cv.GetElemType(cvim)])
        img_msg.step = img_msg.width * cv.CV_MAT_CN(cv.GetElemType(cvim))
        img_msg.data = cvim.tostring()
        return img_msg
