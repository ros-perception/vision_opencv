# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
import rostest
import rospy
import unittest

import sensor_msgs.msg
import warnings

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
        import cv
        self.cvtype_to_name = {}

        for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F" ]:
            for c in [1,2,3,4]:
                nm = "%sC%d" % (t, c)
                self.cvtype_to_name[eval("cv.CV_%s" % nm)] = nm

        self.numpy_type_to_cvtype = {'uint8':'8U', 'int8':'8S', 'uint16':'16U',
                                        'int16':'16S', 'int32':'32S', 'float32':'32F',
                                        'float64':'64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def numpy_type_to_cvtype_with_channels(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)

    def encoding_as_cvtype(self, encoding):
        from cv_bridge.boost.cv_bridge_boost import getCvType

        try:
            res = getCvType(encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def encoding_as_cvtype2(self, encoding):
        from cv_bridge.boost.cv_bridge_boost import getCvType

        try:
            res = getCvType(encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        import re
        vals = re.split('(.+)C(.+)', self.cvtype_to_name[res])
        cvtype = vals[1]
        n_channels = vals[2]

        return self.numpy_type_to_cvtype[cvtype], eval(n_channels)

    def imgmsg_to_cv(self, img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::Image message to an OpenCV :ctype:`IplImage`.

        :param img_msg:   A sensor_msgs::Image message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :ctype:`IplImage`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :ctype:`IplImage` message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        warnings.warn("Will be removed in Indigo. Use imgmsg_to_cv2 instead", DeprecationWarning)
        import cv
        source_type = self.encoding_as_cvtype(img_msg.encoding)
        im = cv.CreateMatHeader(img_msg.height, img_msg.width, source_type)
        cv.SetData(im, img_msg.data, img_msg.step)

        if desired_encoding == "passthrough":
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor

        try:
            res = cvtColor(im, img_msg.encoding, desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def cv_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :ctype:`CvArr` type (that is, an :ctype:`IplImage` or :ctype:`CvMat`) to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :ctype:`IplImage` or :ctype:`CvMat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``

        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns a sensor_msgs::Image message on success, or raises :exc:`opencv_latest.cv_bridge.CvBridgeError` on failure.
        """
        warnings.warn("Will be removed in Indigo. Use cv2_to_imgmsg instead", DeprecationWarning)
        import cv
        img_msg = sensor_msgs.msg.Image()
        (img_msg.width, img_msg.height) = cv.GetSize(cvim)
        if encoding == "passthrough":
            img_msg.encoding = self.cvtype_to_name[cv.GetElemType(cvim)]
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.encoding_as_cvtype(encoding) != cv.GetElemType(cvim):
              raise CvBridgeError, "encoding specified as %s, but image has incompatible type %s" % (encoding, self.cvtype_to_name[cv.GetElemType(cvim)])
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) / img_msg.height
        return img_msg


    def imgmsg_to_cv2(self, img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::Image message to an OpenCV :ctype:`cv::Mat`.

        :param img_msg:   A sensor_msgs::Image message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :ctype:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :ctype:`cv::Mat` message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        import cv2
        import numpy as np
        dtype, n_channels = self.encoding_as_cvtype2(img_msg.encoding)
        im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                           dtype=dtype, buffer=img_msg.data)

        if desired_encoding == "passthrough":
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor2

        try:
            res = cvtColor2(im, img_msg.encoding, desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def cv2_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :ctype:`cv::Mat` type to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :ctype:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``

        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns a sensor_msgs::Image message on success, or raises :exc:`opencv_latest.cv_bridge.CvBridgeError` on failure.
        """
        import cv2
        img_msg = sensor_msgs.msg.Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        cv_type_with_channels = self.numpy_type_to_cvtype_with_channels(cvim.dtype, cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type_with_channels
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.cvtype_to_name[self.encoding_as_cvtype(encoding)] != cv_type_with_channels:
              raise CvBridgeError, "encoding specified as %s, but image has incompatible type %s" % (encoding, cv_type_with_channels)
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) / img_msg.height
        return img_msg
