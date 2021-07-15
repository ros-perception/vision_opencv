# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2016, Tal Regev.
# Copyright (c) 2018 Intel Corporation.
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
#  * Neither the name of the Willow Garage nor the names of its
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
######################################################################

import sys
import time
import math
import rclpy
import numpy
import cv2

import sensor_msgs.msg
from cv_bridge import CvBridge


# Send black pic with a circle as regular and compressed ros msgs.
def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    node = rclpy.create_node("Source")
    node_logger = node.get_logger()

    pub_img = node.create_publisher(sensor_msgs.msg.Image, "/opencv_tests/images")
    pub_compressed_img = node.create_publisher(sensor_msgs.msg.CompressedImage, "/opencv_tests/images/compressed")

    time.sleep(1.0)
    started = time.time()
    counter = 0
    cvim = numpy.zeros((480, 640, 1), numpy.uint8)
    ball_xv = 10
    ball_yv = 10
    ball_x = 100
    ball_y = 100

    cvb = CvBridge()

    while rclpy.ok():
      try:
        cvim.fill(0)
        cv2.circle(cvim, (ball_x, ball_y), 10, 255, -1)

        ball_x += ball_xv
        ball_y += ball_yv
        if ball_x in [10, 630]:
            ball_xv = -ball_xv
        if ball_y in [10, 470]:
            ball_yv = -ball_yv

        pub_img.publish(cvb.cv2_to_imgmsg(cvim))
        pub_compressed_img.publish(cvb.cv2_to_compressed_imgmsg(cvim))
        time.sleep(0.03)
      except KeyboardInterrupt:
        node_logger.info("shutting down: keyboard interrupt")
        break

    node_logger.info("test_completed")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
