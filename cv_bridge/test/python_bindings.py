from nose.tools import assert_equal
import numpy as np

import cv_bridge

import sys
if sys.version > '3':
    xrange = range

def test_cvtColorForDisplay():
    # convert label image to display
    label = np.zeros((480, 640), dtype=np.int32)
    height, width = label.shape[:2]
    label_value = 0
    grid_num_y, grid_num_x = 3, 4
    for grid_row in xrange(grid_num_y):
        grid_size_y = height / grid_num_y
        min_y = grid_size_y * grid_row
        max_y = min_y + grid_size_y
        for grid_col in xrange(grid_num_x):
            grid_size_x = width / grid_num_x
            min_x = grid_size_x * grid_col
            max_x = min_x + grid_size_x
            label[min_y:max_y, min_x:max_x] = label_value
            label_value += 1
    label_viz = cv_bridge.cvtColorForDisplay(label, '32SC1', 'bgr8')
    assert_equal(label_viz.dtype, np.uint8)
    assert_equal(label_viz.min(), 0)
    assert_equal(label_viz.max(), 255)

    # Check that mono8 conversion returns the right shape.
    bridge = cv_bridge.CvBridge()
    mono = np.random.random((100, 100)) * 255
    mono = mono.astype(np.uint8)

    input_msg = bridge.cv2_to_imgmsg(mono, encoding='mono8')
    output = bridge.imgmsg_to_cv2(input_msg, desired_encoding='mono8')
    assert_equal(output.shape, (100,100))
