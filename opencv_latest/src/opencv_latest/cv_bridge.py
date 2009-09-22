import roslib
roslib.load_manifest('opencv_tests')
import rostest
import rospy
import unittest

import sensor_msgs.msg
import cv

class CvBridgeError(TypeError):
    """!
    This is the error raised by CvBridge methods when they fail.
    """
    pass

class CvBridge:

    """!
    @cond DOXYGEN_IGNORE
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

    """!
    @endcond DOXYGEN_IGNORE
    """
    def imgmsg_to_cv(self, img_msg, desired_encoding = "passthrough"):
        """!
        Convert a
        sensor_msgs::Image message to
        an OpenCV IplImage.
        \param img_msg   A sensor_msgs::Image message
        \param desired_encoding  The encoding of the image data, one of the following strings:
           - \c "passthrough"
           - \c "rgb8"
           - \c "rgba8"
           - \c "bgr8"
           - \c "bgra8"
           - \c "mono8"
           - \c "mono16"
            
        If \a desired_encoding is \c "passthrough", then the returned image has the same format as \a img_msg.
        Otherwise \a desired_encoding must be one of the strings \c "rgb8", \c "bgr8", \c "rgba8", \c "bgra8", \c "mono8" or \c "mono16",
        in which case this method converts the image using
        \c CvtColor (http://opencv.willowgarage.com/documentation/python/image_processing.html#CvtColor)
        (if necessary) and the returned image has a type as follows:
           - \c CV_8UC3 (for \c "rgb8", \c "bgr8"),
           - \c CV_8UC4 (for \c "rgba8", \c "bgra8"),
           - \c CV_8UC1 (for \c "mono8"), or
           - \c CV_16UC1 (for \c "mono16")

        This function returns an OpenCV IplImage message on success, or raises CvBridgeError on failure.
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
        """!
        Convert an OpenCV CvArr type (that is, an IplImage or CvMat) to a ROS sensor_msgs::Image message.
        \param cvim      An OpenCV IplImage or CvMat
        \param encoding  The encoding of the image data, one of the following strings:
           - \c "passthrough"
           - \c "rgb8"
           - \c "rgba8"
           - \c "bgr8"
           - \c "bgra8"
           - \c "mono8"
           - \c "mono16"
            
        If \a encoding is \c "passthrough", then the message has the same encoding as the image's OpenCV type.
        Otherwise \a encoding must be one of the strings \c "rgb8", \c "bgr8", \c "rgba8", \c "bgra8", \c "mono8" or \c "mono16",
        in which case the OpenCV image must have the appropriate type:
           - \c CV_8UC3 (for \c "rgb8", \c "bgr8"),
           - \c CV_8UC4 (for \c "rgba8", \c "bgra8"),
           - \c CV_8UC1 (for \c "mono8"), or
           - \c CV_16UC1 (for \c "mono16")

        This function returns a sensor_msgs::Image message on success, or raises CvBridgeError on failure.
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
