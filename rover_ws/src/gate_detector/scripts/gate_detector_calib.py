#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import UInt8MultiArray

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



cv2.namedWindow('Output')
cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
cv2.createTrackbar('Hue_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Hue_High', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Value_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Value_High', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Saturation_Low', 'Trackbars', 0, 255, getTrackValue)
cv2.createTrackbar('Saturation_High', 'Trackbars', 0, 255, getTrackValue)

def gate_detector_calib():


    
    hsvHigh_pub = rospy.Publisher('/gate_detector_calib/hsv_low', UInt8MultiArray, queue_size=1)
    hsvLow_pub = rospy.Publisher('/gate_detector_calib/hsv_low', UInt8MultiArray, queue_size=1)
    rospy.init_node('gate_detector_calib', anonymous=True)
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        hsv_low = np.array((cv2.getTrackbarPos('Hue_Low', 'Trackbars'),
                                        cv2.getTrackbarPos('Saturation_Low', 'Trackbars'),
                                        cv2.getTrackbarPos('Value_Low', 'Trackbars')))

        hsv_high =  np.array((cv2.getTrackbarPos('Hue_High', 'Trackbars'),
                                        cv2.getTrackbarPos('Saturation_High', 'Trackbars'),
                                        cv2.getTrackbarPos('Value_High', 'Trackbars')))

        hsvLow_pub.publish(hsv_low)
        hsvHigh_pub.publish(hsv_high)
        rate.sleep()


if __name__=='__main__':
    try:
        gate_detector_calib()
    except rospy.ROSInterruptException:
        pass

