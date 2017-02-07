
import cv2
import numpy as np
import roslib
import std_msgs.s



def getthresholdedimg(hsv):
    # type: (object) -> object
    threshImg = cv2.inRange(hsv, np.array((cv2.getTrackbarPos('Hue_Low', 'Trackbars'),
                                           cv2.getTrackbarPos('Saturation_Low', 'Trackbars'),
                                           cv2.getTrackbarPos('Value_Low', 'Trackbars'))), np.array((cv2.getTrackbarPos(
        'Hue_High', 'Trackbars'), cv2.getTrackbarPos('Saturation_High', 'Trackbars'), cv2.getTrackbarPos('Value_High',
                                                                                                         'Trackbars'))))

    return threshImg


def getTrackValue(value):
    return value


c = cv2.VideoCapture(0)
width, height = c.get(3), c.get(4)
print "frame width and height : ", width, height

cv2.namedWindow('Output')
cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.createTrackbar('Hue_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Hue_High', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Value_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Value_High', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Saturation_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Saturation_High', 'Trackbars', 0, 255, getTrackValue)

