import array

import cv
import sensor_msgs.msg

def mkmat(rows, cols, L):
    mat = cv.CreateMat(rows, cols, cv.CV_64FC1)
    cv.SetData(mat, array.array('d', L), 8 * cols)
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
        src = mkmat(4, 1, [point[0], point[1], point[2], 1.0])
        dst = cv.CreateMat(3, 1, cv.CV_64FC1)
        cv.MatMul(self.P, src, dst)
        x = dst[0,0]
        y = dst[1,0]
        w = dst[2,0]
        if w != 0:
            return (x / w, y / w)
        else:
            return (0.0, 0.0)

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
        return self.K[0,2]
    def cy(self):
        return self.K[1,2]
    def fx(self):
        return self.K[0,0]
    def fy(self):
        return self.K[1,1]

class StereoCameraModel:
    def __init__(self):
        self.left = PinholeCameraModel()
        self.right = PinholeCameraModel()

    def fromCameraInfo(self, left_msg, right_msg):
        self.left.fromCameraInfo(left_msg)
        self.right.fromCameraInfo(right_msg)

        # [ Fx, 0,  Cx,  Fx*-Tx ]
        # [ 0,  Fy, Cy,  0      ]
        # [ 0,  0,  1,   0      ]
        
        fx = self.right.P[0, 0]
        fy = self.right.P[1, 1]
        cx = self.right.P[0, 2]
        cy = self.right.P[1, 2]
        tx = -self.right.P[0, 3] / fx

        # Q is:
        #    [ 1, 0,  0, -Clx ]
        #    [ 0, 1,  0, -Cy ]
        #    [ 0, 0,  0,  Fx ]
        #    [ 0, 0, 1 / Tx, (Crx-Clx)/Tx ]

        self.Q = cv.CreateMat(4, 4, cv.CV_64FC1)
        cv.SetZero(self.Q)
        self.Q[0, 0] = 1.0
        self.Q[0, 3] = -cx
        self.Q[1, 1] = 1.0
        self.Q[1, 3] = -cy
        self.Q[2, 3] = fx
        self.Q[3, 2] = 1 / tx

    def tfFrame(self):
        return self.left.tfFrame()

    def project3dToPixel(self, point):
        l = self.left.project3dToPixel(point)
        r = self.right.project3dToPixel(point)
        return (l, r)

    def projectPixelTo3d(self, left_uv, disparity):
        src = mkmat(4, 1, [left_uv[0], left_uv[1], disparity, 1.0])
        dst = cv.CreateMat(4, 1, cv.CV_64FC1)
        cv.SetZero(dst)
        cv.MatMul(self.Q, src, dst)
        x = dst[0, 0]
        y = dst[1, 0]
        z = dst[2, 0]
        w = dst[3, 0]
        if w != 0:
            return (x / w, y / w, z / w)
        else:
            return (0.0, 0.0, 0.0)
