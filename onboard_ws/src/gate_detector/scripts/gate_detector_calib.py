#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import UInt8MultiArray
from rover_msgs.msg import HsvVals

class gate_detector_calib:
    def __init__(self):
        self.hsv_pub = rospy.Publisher('/gate_detector_calib/hsv_vals', HsvVals, queue_size=1)
        self.hsv = HsvVals()


    def getthresholdedimg():
        # type: (object) -> object

        cv2.namedWindow('Output')
        cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Hue_Low',         'Trackbars', 0, 255, self.getTrackValue)
        cv2.createTrackbar('Hue_High',        'Trackbars', 0, 255, self.getTrackValue)
        cv2.createTrackbar('Value_Low',       'Trackbars', 0, 255, self.getTrackValue)
        cv2.createTrackbar('Value_High',      'Trackbars', 0, 255, self.getTrackValue)
        cv2.createTrackbar('Saturation_Low',  'Trackbars', 0, 255, self.getTrackValue)
        cv2.createTrackbar('Saturation_High', 'Trackbars', 0, 255, self.getTrackValue)

        hsv.hue_low = cv2.getTrackbarPos('Hue_Low', 'Trackbars')
        hsv.sat_low = cv2.getTrackbarPos('Saturation_Low', 'Trackbars')
        hsv.val_low = cv2.getTrackbarPos('Value_Low', 'Trackbars')

        hsv.hue_high = cv2.getTrackbarPos('Hue_High', 'Trackbars')
        hsv.sat_high = cv2.getTrackbarPos('Saturation_High', 'Trackbars')
        hsv.val_high = cv2.getTrackbarPos('Value_High', 'Trackbars')

        return hsv


    def getTrackValue(self, value):
        return value


    def gate_detector_calib(self):
        self.hsv = getthresholdedimg()


        rospy.init_node('gate_detector_calib', anonymous=True)
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():

            hsv_pub.publish(self.hsv)
            rate.sleep()


if __name__=='__main__':
    try:
        gate_detector_calib()
    except rospy.ROSInterruptException:
        pass
