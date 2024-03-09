from pupil_apriltags import Detection, Detector
import numpy as np 
import cv2

# Need to get all coreners. Remember tag 5 is on the different plane .
tag_ll_corners = [None, 
                np.array([0, 20, 0]),
                None, 
                np.array([0, 0, 0]), 
                None, 
                np.array([-11.3, -4.2, 0]),
                None, 
                np.array([0, 0, -10.8])]

# (Via Elena Giraldo)
def PolyArea2D(pts):
    l = np.hstack([pts, np.roll(pts, -1, axis=0)])
    a = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in l))
    return a

def localize():
    pass 





