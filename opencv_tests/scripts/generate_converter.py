import roslib
roslib.load_manifest('opencv_tests')

import sensor_msgs.msg
import cv

from opencv_latest.cv_bridge import CvBridge, CvBridgeError

def generate_converter():
    map = []
    for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F" ]:
        for c in [1,2,3,4]:
            nm = "%sC%d" % (t, c)
            map.append((nm, nm))
    print "int encoding_as_cvtype(std::string encoding)"
    map.append(("rgb8", "8UC3"))
    map.append(("bgr8", "8UC3"))
    map.append(("rgba8", "8UC4"))
    map.append(("bgra8", "8UC4"))
    map.append(("mono8", "8UC1"))
    map.append(("mono16", "16UC"))
    print "{"
    for (a,b) in map:
        print '  if (encoding == "%s") return CV_%s;' % (a, b)
    print '  return -1;'
    print "}"
    fmts = [ "GRAY", "RGB", "BGR", "RGBA", "BGRA" ]
    print ''
    for src in fmts:
      print '  if (sourcefmt == "%s") {' % src
      for dst in fmts:
        if src != dst:
            print '    if (destfmt == "%s")' % dst
            print '      cvCvtColor(rosimg_, cvtimg_, CV_%s2%s);' % (src, dst)
      print '  }'
    print ''

if __name__ == '__main__':
    generate_converter()
