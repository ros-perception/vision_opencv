import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

class CvBridgeError(TypeError):
    pass

class CvBridge:

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
            return eval("cv.CV_%s" % encoding)

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

        source_type = self.encoding_as_cvtype(img_msg.encoding)
        im = cv.CreateMatHeader(img_msg.height, img_msg.width, source_type)
        cv.SetData(im, img_msg.data, img_msg.step)

        if desired_encoding == "passthrough":
            return im

        # Might need to do a conversion.  sourcefmt and destfmt can be
        # one of GRAY, RGB, BGR, RGBA, BGRA.
        sourcefmt = self.encoding_as_fmt(img_msg.encoding)
        destfmt = self.encoding_as_fmt(desired_encoding)

        destination_type = self.encoding_as_cvtype(desired_encoding)
        if sourcefmt == destfmt and source_type == destination_type:
            return im

        cvtim = cv.CreateMat(img_msg.height, img_msg.width, self.encoding_as_cvtype(desired_encoding))
        if sourcefmt == destfmt:
            cv.ConvertScale(im, cvtim)
        else:
            cv.CvtColor(im, cvtim, eval("cv.CV_%s2%s" % (sourcefmt, destfmt)))
        return cvtim

    def cv_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV CvArr type (that is, an IplImage or CvMat) to a ROS sensor_msgs Image message.
        If encoding is "passthrough", then the message has the same encoding as the image's OpenCV type.
        Otherwise encoding must be one of the defined strings "rgb8", "bgr8", "rgba8", "bgra8", "mono8" or "mono16".
        In this case, the image must have the appropriate type:
           CV_8UC3 (for "rgb8", "bgr8"),
           CV_8UC4 (for "rgba8", "bgra8"),
           CV_8UC1 (for "mono8"), or
           CV_16UC1 (for "mono16").
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
