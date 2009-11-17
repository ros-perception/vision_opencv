import array

import cv
import sensor_msgs.msg

def mkmat(rows, cols, L):
    mat = cv.CreateMat(rows, cols, cv.CV_64FC1)
    cv.SetData(mat, array.array('f', L), 8 * cols)
    return mat

class PinholeCameraModel:

    """
    A pinhole camera is an idealized monocular camera.
    """

    def __init__(self):
        pass

    def fromCameraInfo(self, msg):
        """
        :param msg: camera parameters
        :type msg: sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        self.K = mkmat(3, 3, msg.K)
        self.D = mkmat(4, 1, msg.D[:4])
        self.R = mkmat(3, 3, msg.R)
        self.P = mkmat(3, 4, msg.P)
        self.width = msg.width
        self.height = msg.height

        self.mapx = cv.CreateImage((self.width, self.height), cv.IPL_DEPTH_32F, 1)
        self.mapy = cv.CreateImage((self.width, self.height), cv.IPL_DEPTH_32F, 1)
        cv.InitUndistortMap(self.K, self.D, self.mapx, self.mapy)

    def rectifyImage(self, raw, rectified):
        cv.Remap(raw, rectified, self.mapx, self.mapy)
        
    def rectifyPoint(self, uv_raw):
        src = mkmat(1, 2, list(uv_raw))
        src = cv.Reshape(src, 2)
        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.K, self.D, self.R, self.P)
        return dst[0,0]

    def tfFrame(self):
        pass
    def project3dToPixel(self, point):
        pass
    def projectPixelTo3dRay(self, uv):
        pass

    def intrinsicMatrix(self):
        """ Returns K, also called camera_matrix in cv docs """
        return self.K
    def distortionCoeffs(self):
        """ Returns D """
        return self.D
    def rotationMatrix(self):
        """ Returns R """
        return self.R
    def projectionMatrix(self):
        """ Returns P """
        return self.P

    def cx(self):
        return self.K[0,3]
    def cy(self):
        return self.K[1,3]
    def fx(self):
        return self.K[0,0]
    def fy(self):
        return self.K[1,1]

    """
    // File I/O (using camera_calibration_parsers)
    void load (const std::string& file_name);
    void parse (const std::string& buffer);
    void save (const std::string& file_name) const;
    """

class StereoCameraModel:
    pass
