import gc
import sys

import cv_bridge
from cv_bridge import _cv_bridge
import numpy as np
import pytest


def test_cvtColorForDisplay():
    # convert label image to display
    label = np.zeros((480, 640), dtype=np.int32)
    height, width = label.shape[:2]
    label_value = 0
    grid_num_y, grid_num_x = 3, 4
    for grid_row in range(grid_num_y):
        grid_size_y = height // grid_num_y
        min_y = grid_size_y * grid_row
        max_y = min_y + grid_size_y
        for grid_col in range(grid_num_x):
            grid_size_x = width // grid_num_x
            min_x = grid_size_x * grid_col
            max_x = min_x + grid_size_x
            label[min_y:max_y, min_x:max_x] = label_value
            label_value += 1
    label_viz = cv_bridge.cvtColorForDisplay(label, '32SC1', 'bgr8')
    assert label_viz.dtype == np.uint8
    assert label_viz.min() == 0
    assert label_viz.max() == 255

    # Check that mono8 conversion returns the right shape.
    bridge = cv_bridge.CvBridge()
    mono = np.random.random((100, 100)) * 255
    mono = mono.astype(np.uint8)

    input_msg = bridge.cv2_to_imgmsg(mono, encoding='mono8')
    output = bridge.imgmsg_to_cv2(input_msg, desired_encoding='mono8')
    assert output.shape == (100, 100)


def test_getCvType():
    import cv2
    assert cv_bridge.getCvType('8UC3') == cv2.CV_8UC3
    assert cv_bridge.getCvType('bgr8') == cv2.CV_8UC3
    assert cv_bridge.getCvType('mono16') == cv2.CV_16UC1
    assert cv_bridge.getCvType('32FC1') == cv2.CV_32FC1
    with pytest.raises(RuntimeError):
        cv_bridge.getCvType('not an encoding')


def test_cv_mat_wrappers():
    import cv2
    assert _cv_bridge.CV_MAT_CNWrap(cv2.CV_16UC3) == 3
    assert _cv_bridge.CV_MAT_DEPTHWrap(cv2.CV_16UC3) == cv2.CV_16U


def test_cvtColor2():
    bgr = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)

    rgb = _cv_bridge.cvtColor2(bgr, 'bgr8', 'rgb8')
    assert rgb.dtype == np.uint8
    np.testing.assert_array_equal(rgb, bgr[:, :, ::-1])
    # The input is not modified and the result can be written to.
    np.testing.assert_array_equal(bgr, np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3))
    assert rgb.flags.writeable
    rgb[0, 0, 0] = 255

    # Images with one channel have two dimensions.
    assert _cv_bridge.cvtColor2(bgr, 'bgr8', 'mono8').shape == (4, 6)
    assert _cv_bridge.cvtColor2(bgr, 'bgr8', 'bgra8').shape == (4, 6, 4)

    # The same encoding gives a copy.
    same = _cv_bridge.cvtColor2(bgr, 'bgr8', 'bgr8')
    np.testing.assert_array_equal(same, bgr)
    assert not np.shares_memory(same, bgr)

    with pytest.raises(RuntimeError):
        _cv_bridge.cvtColor2(bgr, 'bgr8', 'bayer_rggb8')


def test_cvtColor2_dtypes():
    for dtype, depth in ((np.uint8, '8U'), (np.int8, '8S'), (np.uint16, '16U'),
                         (np.int16, '16S'), (np.int32, '32S'), (np.float32, '32F'),
                         (np.float64, '64F')):
        for channels in (1, 2, 3, 4):
            shape = (3, 5) if channels == 1 else (3, 5, channels)
            img = (np.arange(np.prod(shape)) % 100).astype(dtype).reshape(shape)
            encoding = '%sC%d' % (depth, channels)
            out = _cv_bridge.cvtColor2(img, encoding, encoding)
            assert out.dtype == dtype, encoding
            np.testing.assert_array_equal(out, img, err_msg=encoding)

    # A third dimension of size one is a single channel.
    assert _cv_bridge.cvtColor2(np.zeros((3, 5, 1), np.uint8), 'mono8', 'mono8').shape == (3, 5)

    mono16 = (np.arange(12, dtype=np.uint16) * 5000).reshape(3, 4)
    np.testing.assert_array_equal(_cv_bridge.cvtColor2(mono16, 'mono16', 'mono8'), mono16 >> 8)


def test_cvtColor2_array_layouts():
    bgr = np.arange(8 * 10 * 3, dtype=np.uint8).reshape(8, 10, 3)

    # Arrays that are not C contiguous
    view = bgr[::2, 1::3]
    assert not view.flags.c_contiguous
    np.testing.assert_array_equal(_cv_bridge.cvtColor2(view, 'bgr8', 'rgb8'), view[:, :, ::-1])
    fortran = np.asfortranarray(bgr)
    np.testing.assert_array_equal(_cv_bridge.cvtColor2(fortran, 'bgr8', 'rgb8'), bgr[:, :, ::-1])

    # Read only arrays
    read_only = bgr.copy()
    read_only.flags.writeable = False
    np.testing.assert_array_equal(_cv_bridge.cvtColor2(read_only, 'bgr8', 'rgb8'),
                                  bgr[:, :, ::-1])


def test_cvtColor2_invalid_arguments():
    for dtype in (bool, np.uint32, np.int64, np.float16):
        with pytest.raises(TypeError):
            _cv_bridge.cvtColor2(np.zeros((4, 4), dtype), 'mono8', 'mono8')
    for shape in ((4,), (2, 2, 2, 2)):
        with pytest.raises(TypeError):
            _cv_bridge.cvtColor2(np.zeros(shape, np.uint8), 'mono8', 'mono8')
    for not_an_array in (None, 1, 'image', [[1, 2], [3, 4]]):
        with pytest.raises(TypeError):
            _cv_bridge.cvtColor2(not_an_array, 'mono8', 'mono8')
    with pytest.raises(RuntimeError):
        _cv_bridge.cvtColor2(np.zeros((0, 0), np.uint8), 'mono8', 'bgr8')


def test_result_outlives_the_input():
    bgr = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    expected = bgr[:, :, ::-1].copy()
    references = sys.getrefcount(bgr)

    rgb = _cv_bridge.cvtColor2(bgr, 'bgr8', 'rgb8')
    # The result does not keep the input alive.
    assert sys.getrefcount(bgr) == references

    del bgr
    gc.collect()
    np.testing.assert_array_equal(rgb, expected)


def test_cvtColorForDisplay_arguments():
    depth = np.array([[0.0, 5.0], [10.0, 20.0]], dtype=np.float32)

    # min_image_value and max_image_value give the range mapped to 0..255.
    fixed = cv_bridge.cvtColorForDisplay(depth, '32FC1', 'mono8', False, 0.0, 20.0)
    np.testing.assert_array_equal(fixed, [[0, 64], [128, 255]])

    # The optional arguments can be given by name
    by_name = cv_bridge.cvtColorForDisplay(
        source=depth, encoding_in='32FC1', encoding_out='mono8',
        min_image_value=0.0, max_image_value=20.0)
    np.testing.assert_array_equal(by_name, fixed)

    # Dynamic scaling uses the range of the image. 0 and 1 are accepted as booleans.
    half = depth[:, :1].copy()
    for enabled in (True, 1):
        scaled = cv_bridge.cvtColorForDisplay(half, '32FC1', 'mono8', enabled)
        np.testing.assert_array_equal(scaled, [[0], [255]])

    # A colormap gives a color image.
    colored = cv_bridge.cvtColorForDisplay(depth, '32FC1', 'bgr8', False, 0.0, 20.0, 2)
    assert colored.shape == (2, 2, 3)
    assert colored.dtype == np.uint8

    with pytest.raises(TypeError):
        cv_bridge.cvtColorForDisplay(depth, '32FC1')


def test_deprecated_boost_module():
    sys.modules.pop('cv_bridge.boost.cv_bridge_boost', None)
    with pytest.warns(DeprecationWarning):
        from cv_bridge.boost import cv_bridge_boost
    for name in ('CV_MAT_CNWrap', 'CV_MAT_DEPTHWrap', 'cvtColor2', 'cvtColorForDisplay',
                 'getCvType'):
        assert getattr(cv_bridge_boost, name) is getattr(_cv_bridge, name)
