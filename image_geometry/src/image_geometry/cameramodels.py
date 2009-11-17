import array

import cv
import sensor_msgs.msg

def mkmat(rows, cols, L):
    mat = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.SetData(mat, array.array('f', L))
    return mat

class PinholeCameraModel:

    def __init__(self):
        pass

    def fromCameraInfo(self, msg):
        self.K = mkmat(3, 3, msg.K)
        self.D = mkmat(4, 1, msg.D[:4])
        self.R = mkmat(3, 3, msg.R)
        self.P = mkmat(3, 4, msg.P)
        self.width = msg.width
        self.height = msg.height



