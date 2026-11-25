import copy
import math

import cv2
import numpy
import numpy as np


def _mkmat(rows, cols, L) -> numpy.ndarray:
    mat = np.array(L, dtype='float64')
    mat.resize(rows, cols)
    return mat


class PinholeCameraModel:
    """A pinhole camera is an idealized monocular camera."""

    def __init__(self):
        self._k = None
        self._d = None
        self._r = None
        self._p = None
        self._full_k = None
        self._full_p = None
        self._width = None
        self._height = None
        self._binning_x = None
        self._binning_y = None
        self._raw_roi = None
        self._tf_frame = None
        self._stamp = None
        self._resolution = None

    def from_camera_info(self, msg) -> None:
        """
        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.

        :param msg: camera parameters
        :type msg:  sensor_msgs.msg.CameraInfo
        """
        self._k = _mkmat(3, 3, msg.k)
        if msg.d:
            self._d = _mkmat(len(msg.d), 1, msg.d)
        else:
            self._d = None
        self._r = _mkmat(3, 3, msg.r)
        self._p = _mkmat(3, 4, msg.p)
        self._full_k = _mkmat(3, 3, msg.k)
        self._full_p = _mkmat(3, 4, msg.p)
        self._width = msg.width
        self._height = msg.height
        self._binning_x = max(1, msg.binning_x)
        self._binning_y = max(1, msg.binning_y)
        self._resolution = (msg.width, msg.height)

        self._raw_roi = copy.copy(msg.roi)
        # ROI all zeros is considered the same as full resolution
        if (self._raw_roi.x_offset == 0 and self._raw_roi.y_offset == 0 and
                self._raw_roi.width == 0 and self._raw_roi.height == 0):
            self._raw_roi.width = self._width
            self._raw_roi.height = self._height
        self._tf_frame = msg.header.frame_id
        self._stamp = msg.header.stamp

        # Adjust K and P for binning and ROI
        self._k[0, 0] /= self._binning_x
        self._k[1, 1] /= self._binning_y
        self._k[0, 2] = (self._k[0, 2] - self._raw_roi.x_offset) / self._binning_x
        self._k[1, 2] = (self._k[1, 2] - self._raw_roi.y_offset) / self._binning_y
        self._p[0, 0] /= self._binning_x
        self._p[1, 1] /= self._binning_y
        self._p[0, 2] = (self._p[0, 2] - self._raw_roi.x_offset) / self._binning_x
        self._p[1, 2] = (self._p[1, 2] - self._raw_roi.y_offset) / self._binning_y
        self._p[0, 3] /= self._binning_x
        self._p[1, 3] /= self._binning_y

    def rectify_image(self, raw, rectified) -> None:
        """
        Rectify the image `raw` and write the result to `rectified`.

        Applies the rectification specified by camera parameters :math:`K` and and :math:`D`.

        `rectified` must have the size of the image the camera publishes, which is the
        region of interest divided by the binning. A :exc:`ValueError` is raised otherwise.

        :param raw:       input image
        :type raw:        :class:`CvMat` or :class:`IplImage`
        :param rectified: rectified output image
        :type rectified:  :class:`CvMat` or :class:`IplImage`
        """
        # K and P are already adjusted for binning and ROI, so the maps must
        # cover the binned ROI that the raw image holds, not the full sensor.
        width = self._raw_roi.width // self._binning_x
        height = self._raw_roi.height // self._binning_y
        self.mapx = numpy.ndarray(shape=(height, width, 1),
                                  dtype='float32')
        self.mapy = numpy.ndarray(shape=(height, width, 1),
                                  dtype='float32')
        cv2.initUndistortRectifyMap(self._k, self._d, self._r, self._p,
                                    (width, height), cv2.CV_32FC1, self.mapx, self.mapy)
        result = cv2.remap(raw, self.mapx, self.mapy, cv2.INTER_CUBIC, rectified)
        if result is not rectified:
            # remap only writes into rectified when its size and type fit the result.
            # Otherwise it returns a new image and leaves rectified untouched.
            if rectified.shape != result.shape:
                raise ValueError(
                    f'rectified has shape {rectified.shape}, '
                    f'but the rectified image has shape {result.shape}')
            rectified[...] = result

    def rectify_point(self, uv_raw) -> numpy.ndarray:
        """
        Return the pixel coordinates of the rectified point.

        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v).

        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)
        :rtype:           numpy.ndarray
        """
        src = _mkmat(1, 2, list(uv_raw))
        src.resize((1, 1, 2))
        dst = cv2.undistortPoints(src, self._k, self._d, R=self._r, P=self._p)
        return dst[0, 0]

    def project_3d_to_pixel(self, point) -> tuple[float, float]:
        """
        Return the rectified pixel coordinates (u, v) of the 3D point.

        Uses the camera :math:`P` matrix.
        This is the inverse of project_pixel_to_3d_ray().

        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:           tuple[float,float]
        """
        src = _mkmat(4, 1, [point[0], point[1], point[2], 1.0])
        dst = self._p @ src
        x = dst[0, 0]
        y = dst[1, 0]
        w = dst[2, 0]
        if w != 0:
            return (x / w, y / w)
        else:
            return (float('nan'), float('nan'))

    def project_pixel_to_3d_ray(self, uv) -> tuple[float, float, float]:
        """
        Return the unit vector from the camera center through rectified pixel (u, v).

        Uses the camera :math:`P` matrix.
        This is the inverse of project_3d_to_pixel().

        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)
        :rtype:           tuple[float,float,float]
        """
        x = (uv[0] - self.cx()) / self.fx()
        y = (uv[1] - self.cy()) / self.fy()
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return (x, y, z)

    def get_delta_u(self, delta_x, z) -> float:
        """
        Compute delta u, given Z and delta X in Cartesian space.

        For given Z, this is the inverse of get_delta_x().

        :param delta_x:         delta X, in cartesian space
        :type delta_x:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float
        """
        if z == 0:
            return float('inf')
        else:
            return self.fx() * delta_x / z

    def get_delta_v(self, delta_y, z) -> float:
        """
        Compute delta v, given Z and delta Y in Cartesian space.

        For given Z, this is the inverse of get_delta_y().

        :param delta_y:         delta Y, in cartesian space
        :type delta_y:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float
        """
        if z == 0:
            return float('inf')
        else:
            return self.fy() * delta_y / z

    def get_delta_x(self, delta_u, z) -> float:
        """
        Compute delta X, given Z in cartesian space and delta u in pixels.

        For given Z, this is the inverse of get_delta_u().

        :param delta_u:         delta u in pixels
        :type delta_u:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float
        """
        return z * delta_u / self.fx()

    def get_delta_y(self, delta_v, z) -> float:
        """
        Compute delta Y, given Z in cartesian space and delta v in pixels.

        For given Z, this is the inverse of get_delta_v().

        :param delta_v:         delta v in pixels
        :type delta_v:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float
        """
        return z * delta_v / self.fy()

    def full_resolution(self) -> tuple[int, int]:
        """
        Return the full resolution of the camera as a tuple in the format (width, height).

        :rtype:                 tuple[int, int]
        """
        return self._resolution

    def intrinsic_matrix(self) -> numpy.ndarray:
        """
        Return :math:`K`, also called camera_matrix in cv docs.

        :rtype:                 numpy.ndarray
        """
        return self._k

    def distortion_coeffs(self) -> numpy.ndarray:
        """
        Return :math:`D`.

        :rtype:                 numpy.ndarray
        """
        return self._d

    def rotation_matrix(self) -> numpy.ndarray:
        """
        Return :math:`R`.

        :rtype:                 numpy.ndarray
        """
        return self._r

    def projection_matrix(self) -> numpy.ndarray:
        """
        Return :math:`P`.

        :rtype:                 numpy.ndarray
        """
        return self._p

    def full_intrinsic_matrix(self) -> numpy.ndarray:
        """
        Return the original camera matrix for full resolution.

        :rtype:                 numpy.ndarray
        """
        return self._full_k

    def full_projection_matrix(self) -> numpy.ndarray:
        """
        Return the projection matrix for full resolution.

        :rtype:                 numpy.ndarray
        """
        return self._full_p

    def cx(self) -> float:
        """
        Return x center.

        :rtype:                 float
        """
        return self._p[0, 2]

    def cy(self) -> float:
        """
        Return y center.

        :rtype:                 float
        """
        return self._p[1, 2]

    def fx(self) -> float:
        """
        Return x focal length.

        :rtype:                 float
        """
        return self._p[0, 0]

    def fy(self) -> float:
        """
        Return y focal length.

        :rtype:                 float
        """
        return self._p[1, 1]

    def tx(self) -> float:
        """
        Return the x-translation term of the projection matrix.

        :rtype:                 float
        """
        return self._p[0, 3]

    def ty(self) -> float:
        """
        Return the y-translation term of the projection matrix.

        :rtype:                 float
        """
        return self._p[1, 3]

    def fov_x(self) -> float:
        """
        Return the horizontal field of view in radians.

        Horizontal FoV = 2 * arctan((width) / (2 * Horizontal Focal Length) )

        :rtype:                 float
        """
        # width is the full resolution, so pair it with the full-resolution focal
        # length: fx() is divided by binning_x.
        return 2 * math.atan(self._width / (2 * self._full_p[0, 0]))

    def fov_y(self) -> float:
        """
        Return the vertical field of view in radians.

        Vertical FoV = 2 * arctan((height) / (2 * Vertical Focal Length) )

        :rtype:                 float
        """
        # height is the full resolution, so pair it with the full-resolution focal
        # length: fy() is divided by binning_y.
        return 2 * math.atan(self._height / (2 * self._full_p[1, 1]))

    def get_tf_frame(self) -> str:
        """
        Return the tf frame name - a string - of the camera.

        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.

        :rtype:                 str
        """
        return self._tf_frame

    def stamp(self):
        """
        Return the time stamp of the :class:`sensor_msgs.msg.CameraInfo` message.

        :rtype:                 builtin_interfaces.msg.Time
        """
        return self._stamp

    def raw_roi(self):
        """
        Return the region of interest of the camera, in the full resolution image.

        A region of interest that is all zeros in the :class:`sensor_msgs.msg.CameraInfo`
        message is returned as the full resolution.

        :rtype:                 sensor_msgs.msg.RegionOfInterest
        """
        return self._raw_roi

    def binning_x(self) -> int:
        """
        Return the number of pixels combined in the horizontal direction.

        :rtype:                 int
        """
        return self._binning_x

    def binning_y(self) -> int:
        """
        Return the number of pixels combined in the vertical direction.

        :rtype:                 int
        """
        return self._binning_y


class StereoCameraModel:
    """An idealized stereo camera."""

    def __init__(self):
        self._left = PinholeCameraModel()
        self._right = PinholeCameraModel()
        self._q = None

    def from_camera_info(self, left_msg, right_msg):
        """
        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` messages.

        :param left_msg: left camera parameters
        :type left_msg:  sensor_msgs.msg.CameraInfo
        :param right_msg: right camera parameters
        :type right_msg:  sensor_msgs.msg.CameraInfo
        """
        self._left.from_camera_info(left_msg)
        self._right.from_camera_info(right_msg)

        # [ Fx, 0,  Cx,  Fx*-Tx ]
        # [ 0,  Fy, Cy,  0      ]
        # [ 0,  0,  1,   0      ]

        assert self._right._p is not None
        fx = self._right.projection_matrix()[0, 0]
        cx = self._right.projection_matrix()[0, 2]
        cy = self._right.projection_matrix()[1, 2]
        tx = -self._right.projection_matrix()[0, 3] / fx

        # Q is:
        #    [ 1, 0,  0, -Clx ]
        #    [ 0, 1,  0, -Cy ]
        #    [ 0, 0,  0,  Fx ]
        #    [ 0, 0, 1 / Tx, (Crx-Clx)/Tx ]

        self._q = numpy.zeros((4, 4), dtype='float64')
        self._q[0, 0] = 1.0
        self._q[0, 3] = -cx
        self._q[1, 1] = 1.0
        self._q[1, 3] = -cy
        self._q[2, 3] = fx
        self._q[3, 2] = 1 / tx

    def get_tf_frame(self) -> str:
        """
        Return the tf frame name - a string - of the camera.

        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.

        :rtype:                 str
        """
        return self._left.get_tf_frame()

    def reprojection_matrix(self) -> numpy.ndarray:
        """
        Return the 4x4 matrix Q that reprojects a disparity image to 3D.

        This is the matrix used by :func:`cv2.reprojectImageTo3D`.

        :rtype:                 numpy.ndarray
        """
        return self._q

    def project_3d_to_pixel(self, point) -> tuple[tuple[float, float], tuple[float, float]]:
        """
        Return the rectified pixel coordinates (u, v) of the 3D point, for each camera.

        The result is ((u_left, v_left), (u_right, v_right)),
        using the cameras' :math:`P` matrices.
        This is the inverse of project_pixel_to_3d().

        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:           tuple[tuple[float,float],tuple[float,float]]
        """
        left = self._left.project_3d_to_pixel(point)
        right = self._right.project_3d_to_pixel(point)
        return (left, right)

    def project_pixel_to_3d(self, left_uv, disparity) -> tuple[float, float, float]:
        """
        Return the 3D point (x, y, z) for the given pixel position.

        Uses the cameras' :math:`P` matrices.
        This is the inverse of project_3d_to_pixel().

        Note that a disparity of zero implies that the 3D point is at infinity.

        :param left_uv:        rectified pixel coordinates
        :type left_uv:         (u, v)
        :param disparity:      disparity, in pixels
        :type disparity:       float
        :rtype:                tuple[float,float,float]
        """
        src = _mkmat(4, 1, [left_uv[0], left_uv[1], disparity, 1.0])
        dst = self._q @ src
        x = dst[0, 0]
        y = dst[1, 0]
        z = dst[2, 0]
        w = dst[3, 0]
        if w != 0:
            return (x / w, y / w, z / w)
        else:
            return (0.0, 0.0, 0.0)

    def get_z(self, disparity) -> float:
        """
        Return the depth at which a point is observed with a given disparity.

        This is the inverse of get_disparity().

        Note that a disparity of zero implies Z is infinite.

        :param disparity:        disparity, in pixels
        :type disparity:         float
        :rtype:                  float
        """
        if disparity == 0:
            return float('inf')
        Tx = -self._right.projection_matrix()[0, 3]
        return Tx / disparity

    def get_disparity(self, z) -> float:
        """
        Return the disparity observed for a point at depth Z.

        This is the inverse of get_z().

        :param z:          Z (depth), in cartesian space
        :type z:           float
        :rtype:            float
        """
        if z == 0:
            return float('inf')
        tx = -self._right.projection_matrix()[0, 3]
        return tx / z

    def get_left_camera(self) -> PinholeCameraModel:
        """
        Return the PinholeCameraModel object of the left camera.

        :rtype: PinholeCameraModel
        """
        return self._left

    def get_right_camera(self) -> PinholeCameraModel:
        """
        Return the PinholeCameraModel object of the right camera.

        :rtype: PinholeCameraModel
        """
        return self._right
