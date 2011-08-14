#!/usr/bin/python

PKG = 'cv_markers' # this package name
import roslib; roslib.load_manifest(PKG)

import math
from functools import partial
import array
import time
import Queue
import threading

import numpy

import rospy
from cv_markers.refinecorners import refinecorners
import cv
import cv_bridge
import sensor_msgs.msg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import geometry_msgs.msg
import tf_conversions.posemath as pm
import PyKDL
import tf.msg

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)

class DataMatrix:
    def __init__(self):
        self.i = 0
        def topic(lr, sub):
            return rospy.resolve_name("stereo") + "/%s/%s" % ({'l' : 'left', 'r' : 'right'}[lr], sub)
        self.tracking = {}
        if 0:
            self.cams = {}
            for lr in "lr":
                rospy.Subscriber(topic(lr, "camera_info"), sensor_msgs.msg.CameraInfo, partial(self.got_info, lr))
            rospy.Subscriber(topic("l", "image_rect_color"), sensor_msgs.msg.Image, self.gotimage)
        else:
            tosync_stereo = [
                (topic('l', "image_rect_color"), sensor_msgs.msg.Image),
                (topic('l', "camera_info"), sensor_msgs.msg.CameraInfo),
                (topic('r', "image_rect_color"), sensor_msgs.msg.Image),
                (topic('r', "camera_info"), sensor_msgs.msg.CameraInfo)
            ]
            tosync_mono = [
                ("image_stream", sensor_msgs.msg.Image),
                ("camera_info", sensor_msgs.msg.CameraInfo),
            ]

            self.q_stereo = Queue.Queue()
            tss = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_stereo], 10)
            tss.registerCallback(self.queue_stereo)

            sth = ConsumerThread(self.q_stereo, self.handle_stereo)
            sth.setDaemon(True)
            sth.start()

            self.q_mono = Queue.Queue()
            tss = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_mono], 10)
            tss.registerCallback(self.queue_mono)

            mth = ConsumerThread(self.q_mono, self.handle_mono)
            mth.setDaemon(True)
            mth.start()

        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)
        self.nf = 0

        # message_filters.Subscriber("/wide_stereo/left/image_raw", sensor_msgs.msg.Image).registerCallback(self.getraw)

    def getraw(self, rawimg):
        print "raw", rawimg.encoding
        rawimg.encoding = "mono8"
        img = CvBridge().imgmsg_to_cv(rawimg)
        cv.SaveImage("/tmp/raw.png", img)

    def got_info(self, lr, cammsg):
        cam = image_geometry.PinholeCameraModel()
        cam.fromCameraInfo(cammsg)
        self.cams[lr] = cam

    def track(self, img):
        self.tracking = {}
        allfound = cv.FindDataMatrix(img)
        # cv.SaveImage("/tmp/ethan%04d.png" % self.nf, img)
        self.nf += 1
        for found in allfound:
            (code,perfect,_) = found
            corners = refinecorners(img, found)
            if corners:
                self.tracking[code] = corners
                # cv.PolyLine(img, [[(int(16*x),int(16*y)) for (x,y) in corners]], True, 0, lineType=cv.CV_AA, shift=4)
        # cv.ShowImage("im", img)
        # cv.WaitKey(6)

    def broadcast(self, header, code, posemsg):
        ts = geometry_msgs.msg.TransformStamped()
        ts.header.frame_id = header.frame_id
        ts.header.stamp = header.stamp
        ts.child_frame_id = code
        ts.transform.translation = posemsg.position
        ts.transform.rotation = posemsg.orientation
        tfm = tf.msg.tfMessage([ts])
        self.pub_tf.publish(tfm)

    def gotimage(self, imgmsg):
        if imgmsg.encoding == "bayer_bggr8":
            imgmsg.encoding = "8UC1"
        img = CvBridge().imgmsg_to_cv(imgmsg)
        # cv.ShowImage("DataMatrix", img)
        # cv.WaitKey(6)

        self.track(img)

        # monocular case
        print self.cams
        if 'l' in self.cams:
            for (code, corners) in self.tracking.items():
                model = cv.fromarray(numpy.array([[0,0,0], [.1, 0, 0], [.1, .1, 0], [0, .1, 0]], numpy.float32))

                rot = cv.CreateMat(3, 1, cv.CV_32FC1)
                trans = cv.CreateMat(3, 1, cv.CV_32FC1)
                cv.FindExtrinsicCameraParams2(model,
                                              cv.fromarray(numpy.array(corners, numpy.float32)),
                                              self.cams['l'].intrinsicMatrix(),
                                              self.cams['l'].distortionCoeffs(),
                                              rot,
                                              trans)

                ts = geometry_msgs.msg.TransformStamped()
                ts.header.frame_id = imgmsg.header.frame_id
                ts.header.stamp = imgmsg.header.stamp
                ts.child_frame_id = code
                posemsg = pm.toMsg(pm.fromCameraParams(cv, rot, trans))
                ts.transform.translation = posemsg.position
                ts.transform.rotation = posemsg.orientation

                tfm = tf.msg.tfMessage([ts])
                self.pub_tf.publish(tfm)

    def queue_mono(self, img, caminfo):
        qq = (img, caminfo)
        self.q_mono.put(qq)

    def queue_stereo(self, lmsg, lcmsg, rmsg, rcmsg):
        qq = (lmsg, lcmsg, rmsg, rcmsg)
        self.q_stereo.put(qq)

    def handle_mono(self, qq):
        (imgmsg, caminfo) = qq
        img = CvBridge().imgmsg_to_cv(imgmsg, "mono8")
        pcm = image_geometry.PinholeCameraModel()
        pcm.fromCameraInfo(caminfo)

        self.track(img)

        for (code, corners) in self.tracking.items():
            # detector numbers vertices clockwise like this:
            # 1 2
            # 0 3
            if isinstance(self.dim, (float, int)):
                d = self.dim
            else:
                d = self.dim[code]
            # User specifies black bar length, but detector uses
            # full module, including the 2-unit quiet zone,
            # so scale by 14/10
            d = d * 14 / 10     
            model = cv.fromarray(numpy.array([[0,0,0], [0, d, 0], [d, d, 0], [d, 0, 0]], numpy.float32))

            rot = cv.CreateMat(3, 1, cv.CV_32FC1)
            trans = cv.CreateMat(3, 1, cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(model,
                                          cv.fromarray(numpy.array(corners, numpy.float32)),
                                          pcm.intrinsicMatrix(),
                                          pcm.distortionCoeffs(),
                                          rot,
                                          trans)

            ts = geometry_msgs.msg.TransformStamped()
            ts.header.frame_id = imgmsg.header.frame_id
            ts.header.stamp = imgmsg.header.stamp
            ts.child_frame_id = code
            posemsg = pm.toMsg(pm.fromCameraParams(cv, rot, trans))
            ts.transform.translation = posemsg.position
            ts.transform.rotation = posemsg.orientation

            tfm = tf.msg.tfMessage([ts])
            self.pub_tf.publish(tfm)

    def handle_stereo(self, qq):

        (lmsg, lcmsg, rmsg, rcmsg) = qq
        limg = CvBridge().imgmsg_to_cv(lmsg, "mono8")
        rimg = CvBridge().imgmsg_to_cv(rmsg, "mono8")

        if 0:
            cv.SaveImage("/tmp/l%06d.png" % self.i, limg)
            cv.SaveImage("/tmp/r%06d.png" % self.i, rimg)
            self.i += 1

        scm = image_geometry.StereoCameraModel()
        scm.fromCameraInfo(lcmsg, rcmsg)

        bm = cv.CreateStereoBMState()
        if "wide" in rospy.resolve_name("stereo"):
            bm.numberOfDisparities = 160
        if 0:
            disparity = cv.CreateMat(limg.rows, limg.cols, cv.CV_16SC1)
            started = time.time()
            cv.FindStereoCorrespondenceBM(limg, rimg, disparity, bm)
            print time.time() - started
            ok = cv.CreateMat(limg.rows, limg.cols, cv.CV_8UC1)
            cv.CmpS(disparity, 0, ok, cv.CV_CMP_GT)
            cv.ShowImage("limg", limg)
            cv.ShowImage("disp", ok)
            cv.WaitKey(6)
        self.track(CvBridge().imgmsg_to_cv(lmsg, "rgb8"))
        if len(self.tracking) == 0:
            print "No markers found"
        for code, corners in self.tracking.items():
            corners3d = []
            for (x, y) in corners:
                limr = cv.GetSubRect(limg, (0, y - bm.SADWindowSize / 2, limg.cols, bm.SADWindowSize + 1))
                rimr = cv.GetSubRect(rimg, (0, y - bm.SADWindowSize / 2, rimg.cols, bm.SADWindowSize + 1))
                tiny_disparity = cv.CreateMat(limr.rows, limg.cols, cv.CV_16SC1)
                cv.FindStereoCorrespondenceBM(limr, rimr, tiny_disparity, bm)
                if tiny_disparity[7, x] < 0:
                    return
                corners3d.append(scm.projectPixelTo3d((x, y), tiny_disparity[7, x] / 16.))
                if 0:
                    cv.ShowImage("d", disparity)
            (a, b, c, d) = [numpy.array(pt) for pt in corners3d]
            def normal(s, t):
                return (t - s) / numpy.linalg.norm(t - s)
            x = PyKDL.Vector(*normal(a, b))
            y = PyKDL.Vector(*normal(a, d))
            f = PyKDL.Frame(PyKDL.Rotation(x, y, x * y), PyKDL.Vector(*a))
            msg = pm.toMsg(f)
            # print "%10f %10f %10f" % (msg.position.x, msg.position.y, msg.position.z)
            print code, msg.position.x, msg.position.y, msg.position.z
            self.broadcast(lmsg.header, code, msg)

def main():
    rospy.init_node('datamatrix')
    node = DataMatrix()
    try:
        node.dim = rospy.get_param('~size')
    except KeyError:
        node.dim = 0.145
    rospy.spin()

if __name__ == "__main__":
    main()
