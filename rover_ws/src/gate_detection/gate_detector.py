
import cv2
import numpy as np


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
cv2.createTrackbar('Caliberate', 'Trackbars', 0, 1, getTrackValue)
cv2.createTrackbar('Box filter', 'Trackbars', 0, 10000, getTrackValue)

while (1):
    grabbed, frame = c.read()
    # this command flips the frame around the y axis
    frame = cv2.flip(frame, 1)

    blur = cv2.medianBlur(frame, 3)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    thrImg = getthresholdedimg(hsv)
    erode = cv2.erode(thrImg, None, iterations=2)
    dilate = cv2.dilate(erode, None, iterations=2)

    contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w / 2, y + h / 2

        if (abs(h * w)**2) > cv2.getTrackbarPos('Box filter', 'Trackbars'):

            cv2.rectangle(frame, (x, y), (x + w, y + h), [0, 0, 255], 2)
            print "******tennis ball detected!!***********"

    if cv2.getTrackbarPos('Caliberate', 'Trackbars') == 1:
        cv2.imshow('Output', thrImg)
    else:
        cv2.imshow('Output', frame)

    # cv2.waitKey(50)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
c.release()

# What would I need to pass to the controller when implementing in ROS for honing?
# - bool: ballFound
# - rectangle size
# - hsv after filtering

