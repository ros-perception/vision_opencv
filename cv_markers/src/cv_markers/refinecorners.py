import time
import itertools

import cv
import numpy

def l2(x0, y0, x1, y1):
    return (x1 - x0) ** 2 + (y1 - y0) ** 2

def strongest(img):
    eig_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)
    temp_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)
    return cv.GoodFeaturesToTrack(img, eig_image, temp_image, 100, 0.03, minDistance = siz / 28, useHarris = True)

siz = 256
sizcorners = [(0,siz-1), (0,0), (siz-1,0), (siz-1,siz-1)]

def backf(hypH, im, found):
    (code,corners,pattern) = found
    persp = cv.CreateMat(3, 3, cv.CV_32FC1)
    fc = [corners[i,0] for i in range(4)]
    cv.GetPerspectiveTransform(fc, sizcorners, persp)
    cc = cv.Reshape(cv.fromarray(numpy.array(sizcorners).astype(numpy.float32)), 2)
    t1 = cv.CreateMat(4, 1, cv.CV_32FC2)
    t2 = cv.CreateMat(4, 1, cv.CV_32FC2)
    _persp = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.Invert(persp, _persp)
    _hypH = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.Invert(hypH, _hypH)

    cv.PerspectiveTransform(cc, t1, _hypH)
    cv.PerspectiveTransform(t1, t2, _persp)
    return [t2[i,0] for i in range(4)]
# im = cv.GetMat(cv.LoadImage("000000.png", 0))
def refinecorners(im, found):
    """ For a found marker, return the refined corner positions """
    t0 = time.time()
    (code,corners,pattern) = found
    persp = cv.CreateMat(3, 3, cv.CV_32FC1)
    fc = [corners[i,0] for i in range(4)]
    cv.GetPerspectiveTransform(fc, sizcorners, persp)
    cim = cv.CreateMat(siz, siz, cv.CV_8UC1)
    cv.WarpPerspective(im, cim, persp, flags = cv.CV_INTER_LINEAR|cv.CV_WARP_FILL_OUTLIERS, fillval = 255)

    unit = siz / 14.
    hunit = unit / 2
    def nearest1(x, y):
        ix = int((x + hunit) / unit)
        iy = int((y + hunit) / unit)
        if (2 <= ix < 13) and (2 <= iy < 13):
            nx = int(unit * ix)
            ny = int(unit * iy)
            return (nx, ny)
        else:
            return (0,0)

    def nearest(x, y):
        """ Return all grid points within sqrt(2) units of (x,y), closest first """
        close = []
        for ix in range(2, 14):
            for iy in range(2, 14):
                (nx, ny) = (unit * ix, unit * iy)
                d = l2(x, y, nx, ny)
                close.append((d, (nx, ny)))
        return [p for (d,p) in sorted(close) if d < 2*unit*unit]

    corners = strongest(cim)
    pool = [((x,y), nearest(x, y)) for (x, y) in corners]

    ga = dict([(x+y,((x,y),P)) for ((x,y),P) in pool])
    gb = dict([(x-y,((x,y),P)) for ((x,y),P) in pool])
    hyp = [ga[min(ga)], ga[max(ga)],
           gb[min(gb)], gb[max(gb)]]

    aL = [a for (a,bs) in hyp]
    oldcorners = cv.fromarray(numpy.array(corners).astype(numpy.float32))
    oldcorners = cv.Reshape(oldcorners, 2)
    newcorners = cv.CreateMat(len(corners), 1, cv.CV_32FC2)
    best = (9999999, None)
    for bL in itertools.product(*[bs for (a,bs) in hyp]):
        hypH = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.GetPerspectiveTransform(aL, bL, hypH)
        cv.PerspectiveTransform(oldcorners, newcorners, hypH)
        error = 0
        for i in range(newcorners.rows):
            (x,y) = newcorners[i,0]
            (nx, ny) = nearest1(x, y)
            error += l2(x, y, nx, ny)
        best = min(best, (error, hypH))
        if error < 1000:
            break
    # print "took", time.time() - t0, best[0]
    if best[0] < 2500:
        pose = best[1]
        return backf(pose, im, found)
    else:
        return None

