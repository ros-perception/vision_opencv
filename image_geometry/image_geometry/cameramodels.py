import cv2
import math
import copy
import numpy
import numpy as np
import warnings
from deprecated.sphinx import deprecated

def _mkmat(rows, cols, L) -> numpy.ndarray:
    mat = np.array(L,dtype='float64')
    mat.resize(rows,cols)
    return mat

class PinholeCameraModel:

    """
    A pinhole camera is an idealized monocular camera.
    """

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

    def from_camera_info(self, msg)->None:
        """
        :param msg: camera parameters
        :type msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.
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
        self._k[0,0] /= self._binning_x
        self._k[1,1] /= self._binning_y
        self._k[0,2] = (self._k[0,2] - self._raw_roi.x_offset) / self._binning_x
        self._k[1,2] = (self._k[1,2] - self._raw_roi.y_offset) / self._binning_y
        self._p[0,0] /= self._binning_x
        self._p[1,1] /= self._binning_y
        self._p[0,2] = (self._p[0,2] - self._raw_roi.x_offset) / self._binning_x
        self._p[1,2] = (self._p[1,2] - self._raw_roi.y_offset) / self._binning_y

    def rectify_image(self, raw, rectified)->None:
        """
        :param raw:       input image
        :type raw:        :class:`CvMat` or :class:`IplImage`
        :param rectified: rectified output image
        :type rectified:  :class:`CvMat` or :class:`IplImage`

        Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
        """

        self.mapx = numpy.ndarray(shape=(self._height, self._width, 1),
                           dtype='float32')
        self.mapy = numpy.ndarray(shape=(self._height, self._width, 1),
                           dtype='float32')
        cv2.initUndistortRectifyMap(self._k, self._d, self._r, self._p,
                (self._width, self._height), cv2.CV_32FC1, self.mapx, self.mapy)
        cv2.remap(raw, self.mapx, self.mapy, cv2.INTER_CUBIC, rectified)

    def rectify_point(self, uv_raw)->numpy.ndarray:
        """
        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)
        :rtype:           numpy.ndarray

        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v) and returns the
        pixel coordinates of the rectified point.
        """

        src = _mkmat(1, 2, list(uv_raw))
        src.resize((1,1,2))
        dst = cv2.undistortPoints(src, self._k, self._d, R=self._r, P=self._p)
        return dst[0,0]
    
    def project_3d_to_pixel(self, point)->tuple[float,float]:
        """
        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:           tuple[float,float]

        Returns the rectified pixel coordinates (u, v) of the 3D point,
        using the camera :math:`P` matrix.
        This is the inverse of project_pixel_to_3d_ray().
        """
        src = _mkmat(4, 1, [point[0], point[1], point[2], 1.0])
        dst = self._p @ src
        x = dst[0,0]
        y = dst[1,0]
        w = dst[2,0]
        if w != 0:
            return (x / w, y / w)
        else:
            return (float('nan'), float('nan'))
    
    def project_pixel_to_3d_ray(self, uv)->tuple[float,float,float]:
        """
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)
        :rtype:           tuple[float,float,float]

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of project_3d_to_pixel().
        """
        x = (uv[0] - self.cx()) / self.fx()
        y = (uv[1] - self.cy()) / self.fy()
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return (x, y, z)

    def get_delta_u(self, delta_x, z)->float:
        """
        :param delta_x:         delta X, in cartesian space
        :type delta_x:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float

        Compute delta u, given Z and delta X in Cartesian space.
        For given Z, this is the inverse of get_delta_x().
        """
        if z == 0:
            return float('inf')
        else:
            return self.fx() * delta_x / z

    def get_delta_v(self, delta_y, z)->float:
        """
        :param delta_y:         delta Y, in cartesian space
        :type delta_y:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float

        Compute delta v, given Z and delta Y in Cartesian space.
        For given Z, this is the inverse of get_delta_y().
        """
        if z == 0:
            return float('inf')
        else:
            return self.fy() * delta_y / z

    def get_delta_x(self, delta_u, z)->float:
        """
        :param deltaU:          delta u in pixels
        :type deltaU:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta X, given Z in cartesian space and delta u in pixels.
        For given Z, this is the inverse of get_delta_u().
        """
        return z * delta_u / self.fx()

    def get_delta_y(self, delta_v, z)->float:
        """
        :param delta_v:         delta v in pixels
        :type delta_v:          float
        :param z:               Z, in cartesian space
        :type z:                float
        :rtype:                 float

        Compute delta Y, given Z in cartesian space and delta v in pixels.
        For given Z, this is the inverse of get_delta_v().
        """
        return z * delta_v / self.fy()

    def full_resolution(self)->tuple[int, int]:
        """
        :rtype:                 tuple[int, int]

        Returns the full resolution of the camera as a tuple in the format (width, height)
        """
        return self._resolution

    def intrinsic_matrix(self)->numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray

        Returns :math:`K`, also called camera_matrix in cv docs 
        """
        return self._k

    def distortion_coeffs(self)->numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray
        
        Returns :math:`D` 
        """
        return self._d
    
    def rotation_matrix(self)->numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray

        Returns :math:`R` 
        """
        return self._r
    
    def projection_matrix(self) ->numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray

        Returns :math:`P` 
        """
        return self._p
    
    def full_intrinsic_matrix(self) -> numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray

        Return the original camera matrix for full resolution 
        """
        return self._full_k

    def full_projection_matrix(self)->numpy.ndarray:
        """ 
        :rtype:                 numpy.ndarray

        Return the projection matrix for full resolution """
        return self._full_p

    def cx(self)->float:
        """ 
        :rtype:                 float      
        
        Returns x center """
        return self._p[0,2]

    def cy(self)->float:
        """ 
        :rtype:                 float      
        
        Returns y center 
        """
        return self._p[1,2]

    def fx(self)->float:
        """ 
        :rtype:                 float      
        
        Returns x focal length 
        """
        return self._p[0,0]

    def fy(self)->float:
        """ 
        :rtype:                 float      
        
        Returns y focal length 
        """
        return self._p[1,1]

    def tx(self)->float:
        """ 
        :rtype:                 float      
        
        Return the x-translation term of the projection matrix 
        """
        return self._p[0,3]

    def ty(self)->float:
        """ 
        :rtype:                 float      
        
        Return the y-translation term of the projection matrix 
        """
        return self._p[1,3]
    
    def fov_x(self)->float:
        """ 
        :rtype:                 float      
        
        Returns the horizontal field of view in radians.
        Horizontal FoV = 2 * arctan((width) / (2 * Horizontal Focal Length) )
        """
        # width is the full resolution, so pair it with the full-resolution focal
        # length: fx() is divided by binning_x.
        return 2 * math.atan(self._width / (2 * self._full_p[0, 0]))


    def fov_y(self)->float:
        """ 
        :rtype:                 float      
        
        Returns the vertical field of view in radians.
        Vertical FoV = 2 * arctan((height) / (2 * Vertical Focal Length) )
        """
        # height is the full resolution, so pair it with the full-resolution focal
        # length: fy() is divided by binning_y.
        return 2 * math.atan(self._height / (2 * self._full_p[1, 1]))

    def get_tf_frame(self)->str:
        """ 
        :rtype:                 str      
        
        Returns the tf frame name - a string - of the camera.
        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        return self._tf_frame

    # Deprecated members planned for removal in K-turtle
    # --------------------------------------------------
    @property
    @deprecated(version="J-turtle", reason="The binning_x property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def binning_x(self):
        """ 
        .. warning::
            The binning_x property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._binning_x

    @property
    @deprecated(version="J-turtle", reason="The binning_y property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def binning_y(self):
        """ 
        .. warning::
            The binning_y property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._binning_y

    @property
    @deprecated(version="J-turtle", reason="The D->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the distortion_coeffs()->numpy.ndarray method instead.")
    def D(self)->numpy.matrix:
        """ 
        .. warning::
            The D property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._d, dtype="float64")

    @property
    @deprecated(version="J-turtle", reason="The full_K->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_intrinsic_matrix()->numpy.ndarray method instead.")
    def full_K(self)->numpy.matrix:
        """ 
        .. warning::
            The full_K property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._full_k, dtype="float64")

    @property
    @deprecated(version="J-turtle", reason="The full_P->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_projection_matrix()->numpy.ndarray method instead.")
    def full_P(self)->numpy.matrix:
        """ 
        .. warning::
            The full_P property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._full_p, dtype="float64")

    @property
    @deprecated(version="J-turtle", reason="The height property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def height(self):
        """ 
        .. warning::
            The height property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._height

    @property
    @deprecated(version="J-turtle", reason="The K->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the intrinsic_matrix()->numpy.ndarray method instead.")
    def K(self)->numpy.matrix:
        """ 
        .. warning::
            The K property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._k, dtype="float64")

    @property
    @deprecated(version="J-turtle", reason="The P->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the projection_matrix()->numpy.ndarray method instead.")
    def P(self)->numpy.matrix:
        """ 
        .. warning::
            The P property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._p, dtype="float64")

    @property
    @deprecated(version="J-turtle", reason="The R->numpy.matrix property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rotation_matrix()->numpy.ndarray method instead.")
    def R(self)->numpy.matrix:
        """ 
        .. warning::
            The R property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return numpy.matrix(self._r)

    @property
    @deprecated(version="J-turtle", reason="The binning_y property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def raw_roi(self):
        """ 
        .. warning::
            The raw_roi property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._raw_roi

    @property
    @deprecated(version="J-turtle", reason="The stamp property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def stamp(self):
        """ 
        .. warning::
            The stamp property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._stamp

    @property
    @deprecated(version="J-turtle", reason="The tf_frame property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_tf_frame() method.")
    def tf_frame(self):
        """ 
        .. warning::
            The tf_frame property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._tf_frame

    @property
    @deprecated(version="J-turtle", reason="The width property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def width(self):
        """ 
        .. warning::
            The width property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._width
    
    @deprecated(version="J-turtle", reason="The distortionCoeffs()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the distortion_coeffs()->numpy.ndarray method instead.")
    def distortionCoeffs(self)->numpy.matrix:
        """ 
        .. warning::
            The distortionCoeffs()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the distortion_coeffs()->numpy.ndarray method instead."
        
        :rtype:                 numpy.matrix
        
        Returns :math:`D` 
        """
        return numpy.matrix(self.distortion_coeffs(), dtype="float64")

    @deprecated(version="J-turtle", reason="The fovX() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the fov_x() method instead.")
    def fovX(self)->float:
        """ 
        .. warning::
            The fovX() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the fov_x() method instead.

        :rtype:                 float      
        
        Returns the horizontal field of view in radians.
        Horizontal FoV = 2 * arctan((width) / (2 * Horizontal Focal Length) )
        """
        return self.fov_x()

    @deprecated(version="J-turtle", reason="The fovY() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the fov_y() method instead.")
    def fovY(self)->float:
        """ 
        .. warning::
            The fovY() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the fov_y() method instead.
        
        :rtype:                 float      
        
        Returns the vertical field of view in radians.
        Vertical FoV = 2 * arctan((height) / (2 * Vertical Focal Length) )
        """
        return self.fov_y()

    @deprecated(version="J-turtle", reason="The fromCameraInfo() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the from_camera_info() method instead.")
    def fromCameraInfo(self,msg)->None:
        """
        .. warning::
            The fromCameraInfo() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the from_camera_info() method instead.

        :param msg: camera parameters
        :type msg:  sensor_msgs.msg.CameraInfo
        
        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        self.from_camera_info(msg)

    @deprecated(version="J-turtle", reason="The fullIntrinsicMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_intrinsic_matrix()->numpy.ndarray method instead.")
    def fullIntrinsicMatrix(self) -> numpy.matrix:
        """ 
        .. warning::
            The fullIntrinsicMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_intrinsic_matrix()->numpy.ndarray method instead.
        
        :rtype:                 numpy.matrix

        Return the original camera matrix for full resolution 
        """
        return numpy.matrix(self.full_intrinsic_matrix(), dtype='float64')

    @deprecated(version="J-turtle", reason="The fullProjectionMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_projection_matrix()->numpy.ndarray method instead.")
    def fullProjectionMatrix(self)->numpy.matrix:
        """ 
        .. warning::
            The fullProjectionMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_projection_matrix()->numpy.ndarray method instead.
        
        :rtype:                 numpy.matrix

        Return the projection matrix for full resolution """
        return numpy.matrix(self.full_projection_matrix(), dtype='float64')
 
    @deprecated(version="J-turtle", reason="The fullResolution() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_resolution() method instead.")
    def fullResolution(self)->tuple[int, int]:
        """
        .. warning::
            The fullResolution() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the full_resolution() method instead.
            
        :rtype:                 tuple[int, int]

        Returns the full resolution of the camera as a tuple in the format (width, height)
        """
        return self.full_resolution()

    @deprecated(version="J-turtle", reason="The getDeltaU() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_u() method instead.")
    def getDeltaU(self, deltaX, Z)->float:
        """
        .. warning::
            The getDeltaU() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_u() method instead.
        
        :param deltaX:          delta X, in cartesian space
        :type deltaX:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta u, given Z and delta X in Cartesian space.
        For given Z, this is the inverse of :math:`get_delta_x`.
        """
        return self.get_delta_u(deltaX, Z)

    @deprecated(version="J-turtle", reason="The getDeltaV() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_v() method instead.")
    def getDeltaV(self, deltaY, Z)->float:
        """
        .. warning::
            he getDeltaV() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_v() method instead.

        :param deltaY:          delta Y, in cartesian space
        :type deltaY:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta v, given Z and delta Y in Cartesian space.
        For given Z, this is the inverse of :math:`get_delta_y`.

        """
        return(self.get_delta_v(deltaY,Z))    

    @deprecated(version="J-turtle", reason="The getDeltaX() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_x() method instead.")
    def getDeltaX(self, deltaU, Z)->float:
        """
        .. warning::
            The getDeltaX() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_x() method instead.
        
        :param deltaU:          delta u in pixels
        :type deltaU:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta X, given Z in cartesian space and delta u in pixels.
        For given Z, this is the inverse of :math:`get_delta_u`.
        """
        return self.get_delta_x(deltaU,Z)
    
    @deprecated(version="J-turtle", reason="The getDeltaY() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_y() method instead.")
    def getDeltaY(self, deltaV, Z)->float:
        """
        .. warning::
            The getDeltaY() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_delta_y() method instead.

        :param deltaV:          delta v in pixels
        :type deltaV:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta Y, given Z in cartesian space and delta v in pixels.
        For given Z, this is the inverse of :math:`get_delta_v`.
        """
        return self.get_delta_y(deltaV,Z)

    @deprecated(version="J-turtle", reason="The intrinsicMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the intrinsic_matrix()->numpy.ndarray method instead.")
    def intrinsicMatrix(self)->numpy.matrix:
        """ 
        .. warning::
            The intrinsicMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the intrinsic_matrix()->numpy.ndarray method instead.
        
        :rtype:                 numpy.matrix

        Returns :math:`K`, also called camera_matrix in cv docs 
        """
        return numpy.matrix(self.intrinsic_matrix(), dtype="float64")

    @deprecated(version="J-turtle", reason="The project3dToPixel() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_3d_to_pixel() method instead.")
    def project3dToPixel(self, point)->tuple[float,float]:
        """
        .. warning::
            The project3dToPixel() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_3d_to_pixel() method instead.

        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:            tuple[float,float]
        
        Returns the rectified pixel coordinates (u, v) of the 3D point,
        using the camera :math:`P` matrix.
        This is the inverse of :math:`projectPixelTo3dRay`.
        """
        return self.project_3d_to_pixel(point)
    
    @deprecated(version="J-turtle", reason="The projectionMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the projection_matrix()->numpy.ndarray method instead.")
    def projectionMatrix(self) -> numpy.matrix:
        """ 
        .. warning::
            The projectionMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the projection_matrix()->numpy.ndarray method instead.
        
        :rtype:                 numpy.matrix

        Returns :math:`P` 
        """
        return np.matrix(self.projection_matrix(), dtype='float64')

    @deprecated(version="J-turtle", reason="The projectPixelTo3dRay() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_pixel_to_3d_ray() method instead.")
    def projectPixelTo3dRay(self, uv)->tuple[float,float,float]:
        """
        .. warning::
            The projectPixelTo3dRay() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_pixel_to_3d_ray() method instead.
        
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)
        :rtype:           tuple[float,float,float]

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of :math:`project_3d_to_pixel`.
        """
        return self.project_pixel_to_3d_ray(uv)

    @deprecated(version="J-turtle", reason="The rectifyImage() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rectify_image() method instead.")
    def rectifyImage(self, raw, rectified):
        """
        .. warning::
            The rectifyImage() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rectify_image() method instead.

        :param raw:       input image
        :type raw:        :class:`CvMat` or :class:`IplImage`
        :param rectified: rectified output image
        :type rectified:  :class:`CvMat` or :class:`IplImage`
        
        Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
        """
        self.rectify_image(raw, rectified)

    @deprecated(version="J-turtle", reason="The rectifyPoint() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rectify_point() method instead.")
    def rectifyPoint(self, uv_raw)->numpy.ndarray:
        """
        .. warning::
            The rectifyPoint() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rectify_point() method instead.
        
        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)
        :rtype:           numpy.ndarray
        
        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v) and returns the
        pixel coordinates of the rectified point.
        """
        return self.rectify_point(uv_raw)

    @deprecated(version="J-turtle", reason="The rotationMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rotation_matrix()->numpy.ndarray method instead.")
    def rotationMatrix(self)->numpy.matrix:
        """ 
        .. warning::
            The rotationMatrix()->numpy.matrix method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the rotation_matrix()->numpy.ndarray method instead.
        
        :rtype:                 numpy.matrix

        Returns :math:`R` 
        """
        return np.matrix(self.rotation_matrix(), dtype='float64')

    @deprecated(version="J-turtle", reason="The tfFrame() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_tf_frame() method instead.")
    def tfFrame(self)->str:
        """ 
        .. warning::
            The tfFrame() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_tf_frame() method instead.

        :rtype:                 str      
        
        Returns the tf frame name - a string - of the camera.
        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        return self.get_tf_frame()

    @deprecated(version="J-turtle", reason="The Tx() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the tx() method instead.")
    def Tx(self)->float:
        """ 
        .. warning::
            The Tx() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the tx() method instead.
        
        :rtype:                 float      

        Return the x-translation term of the projection matrix 
        """
        return self.tx()

    @deprecated(version="J-turtle", reason="The Ty() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the ty() method instead.")
    def Ty(self)->float:
        """ 
        .. warning::
            The Ty() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the ty() method instead.
        
        :rtype:                 float      
        
        Return the y-translation term of the projection matrix 
        """
        return self.ty()




class StereoCameraModel:
    """
    An idealized stereo camera.
    """
    def __init__(self):
        self._left = PinholeCameraModel()
        self._right = PinholeCameraModel()
        self._q = None
    
    def from_camera_info(self, left_msg, right_msg):
        """
        :param left_msg: left camera parameters
        :type left_msg:  sensor_msgs.msg.CameraInfo
        :param right_msg: right camera parameters
        :type right_msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` messages.
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

    def get_tf_frame(self)->str:
        """ 
        :rtype:                 str      
        
        Returns the tf frame name - a string - of the camera.
        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        return self._left.get_tf_frame()

    def project_3d_to_pixel(self, point)->tuple[tuple[float,float],tuple[float,float]]:
        """
        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:           tuple[tuple[float,float],tuple[float,float]]
        
        Returns the rectified pixel coordinates (u, v) of the 3D point, for each camera, as ((u_left, v_left), (u_right, v_right))
        using the cameras' :math:`P` matrices.
        This is the inverse of project_pixel_to_3d().
        """
        l = self._left.project_3d_to_pixel(point)
        r = self._right.project_3d_to_pixel(point)
        return (l, r)

    def project_pixel_to_3d(self, left_uv, disparity)->tuple[float,float,float]:
        """
        :param left_uv:        rectified pixel coordinates
        :type left_uv:         (u, v)
        :param disparity:      disparity, in pixels
        :type disparity:       float
        :rtype:                tuple[float,float,float] 

        Returns the 3D point (x, y, z) for the given pixel position,
        using the cameras' :math:`P` matrices.
        This is the inverse of project_3d_to_pixel().

        Note that a disparity of zero implies that the 3D point is at infinity.
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

    def get_z(self, disparity)->float:
        """
        :param disparity:        disparity, in pixels
        :type disparity:         float
        :rtype:                  float

        Returns the depth at which a point is observed with a given disparity.
        This is the inverse of get_disparity().

        Note that a disparity of zero implies Z is infinite.
        """
        if disparity == 0:
            return float('inf')
        Tx = -self._right.projection_matrix()[0, 3]
        return Tx / disparity

    def get_disparity(self, z)->float:
        """
        :param z:          Z (depth), in cartesian space
        :type z:           float
        :rtype:            float

        Returns the disparity observed for a point at depth Z.
        This is the inverse of get_z().
        """
        if z == 0:
            return float('inf')
        tx = -self._right.projection_matrix()[0, 3]
        return tx / z
        
    def get_left_camera(self)->PinholeCameraModel:
        """ 
        :rtype: PinholeCameraModel

        Returns the PinholeCameraModel object of the left camera
        """
        return self._left
    
    def get_right_camera(self)->PinholeCameraModel:
        """ 
        :rtype: PinholeCameraModel

        Returns the PinholeCameraModel object of the right camera
        """
        return self._right
    
    # Deprecated members planned for removal in K-turtle
    # --------------------------------------------------
    @property
    @deprecated(version="J-turtle", reason="The left property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_left_camera() method.")
    def left(self):
        """ 
        .. warning::
            The left property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._left

    @property
    @deprecated(version="J-turtle", reason="The right property is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_right_camera() method.")
    def right(self):
        """ 
        .. warning::
            The right property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._right

    @property
    @deprecated(version="J-turtle", reason="The Q property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.")
    def Q(self):
        """ 
        .. warning::
            The Q property is deprecated as of J-turtle, and will be removed in K-turtle. It is not meant to be an exposed member.
        """
        return self._q

    @deprecated(version="J-turtle", reason="The fromCameraInfo() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the from_camera_info() method instead.")
    def fromCameraInfo(self, left_msg, right_msg):
        """
        .. warning::
            The fromCameraInfo() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the from_camera_info() method instead.
        
        :param left_msg: left camera parameters
        :type left_msg:  sensor_msgs.msg.CameraInfo
        :param right_msg: right camera parameters
        :type right_msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` messages.
        """

        self.from_camera_info(left_msg,right_msg)

    @deprecated(version="J-turtle", reason="The getDisparity() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_disparity() method instead.")
    def getDisparity(self, Z)->float:
        """
        .. warning::
            The getDisparity() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_disparity() method instead.
        
        :param Z:          Z (depth), in cartesian space
        :type Z:           float
        :rtype:            float

        Returns the disparity observed for a point at depth Z.
        This is the inverse of :math:`getZ`.
        """
        return self.get_disparity(Z)
    
    @deprecated(version="J-turtle", reason="The getZ() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_z() method instead.")
    def getZ(self, disparity)->float:
        """
        .. warning::
            The getZ() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_z() method instead.
        
        :param disparity:        disparity, in pixels
        :type disparity:         float
        :rtype:                  float

        Returns the depth at which a point is observed with a given disparity.
        This is the inverse of :math:`getDisparity`.

        Note that a disparity of zero implies Z is infinite.
        """
        return self.get_z(disparity)

    @deprecated(version="J-turtle", reason="The project3dToPixel() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_3d_to_pixel() method instead.")
    def project3dToPixel(self, point)->tuple[tuple[float,float],tuple[float,float]]:
        """
        .. warning::
            The project3dToPixel() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_3d_to_pixel() method instead.

        :param point:     3D point
        :type point:      (x, y, z)
        :rtype:           tuple[tuple[float,float],tuple[float,float]]
        
        Returns the rectified pixel coordinates (u, v) of the 3D point, for each camera, as ((u_left, v_left), (u_right, v_right))
        using the cameras' :math:`P` matrices.
        This is the inverse of :math:`projectPixelTo3d`.
        """
        return self.project_3d_to_pixel(point)

    @deprecated(version="J-turtle", reason="The projectPixelTo3d() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_pixel_to_3d() method instead.")
    def projectPixelTo3d(self, left_uv, disparity)->tuple[float,float,float]:
        """
        .. warning::
            The projectPixelTo3d() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the project_pixel_to_3d() method instead.
        
        :param left_uv:        rectified pixel coordinates
        :type left_uv:         (u, v)
        :param disparity:      disparity, in pixels
        :type disparity:       float
        :rtype:                tuple[float,float,float] 

        Returns the 3D point (x, y, z) for the given pixel position,
        using the cameras' :math:`P` matrices.
        This is the inverse of :math:`project3dToPixel`.

        Note that a disparity of zero implies that the 3D point is at infinity.
        """
        return self.project_pixel_to_3d(left_uv,disparity)

    @deprecated(version="J-turtle", reason="The tfFrame() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the tf_frame() method instead.")
    def tfFrame(self)->str:
        """ 
        .. warning::
            The tfFrame() method is deprecated as of J-turtle, and will be removed in K-turtle. Please use the get_tf_frame() method instead.
        
        :rtype:                 str      
        
        Returns the tf frame name - a string - of the camera.
        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        return self.get_tf_frame()

