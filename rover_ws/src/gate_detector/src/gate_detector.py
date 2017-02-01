#!/usr/bin/env python

import roslib
import cv2
import numpy as np
import rospy, sys, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class gate_detector:
    def __init__(self):

        self.detector_pub = rospy.Publisher("gate_detector/image_detector", Image, queue_size=10)
        self.hsv_pub = rospy.Publisher("gate_detector/hsv", Image, queue_size=10)
        self.isDetected_pub = rospy.Publisher("gate_detector/isDetected", Bool, queue_size = 1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        self._hl = rospy.get_param('_hue_lower', 28)
        self._hu = rospy.get_param('_hue_upper', 54)
        self._sl = rospy.get_param('_sat_lower', 136)
        self._su = rospy.get_param('_sat_upper', 255)
        self._vl = rospy.get_param('_val_lower', 111)
        self._vu = rospy.get_param('_val_upper', 255)
        self.isDetected = False


    def callback(self, data):
        
        print('getting here')
        img_msg = data
        print("got data")
        try:
            print("got the cv_image")
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            print("goterror")
            print(e)

        # cv_image = cv2.cvtColor(cv_image, cv2.)    
        # this command flips the frame around the y axis

        frame = cv_image
        lower =  np.array([self._hl, self._sl, self._vl])
        upper =  np.array([self._hu, self._su, self._vu])
        blur = cv2.medianBlur(cv_image, 3)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        thrImg = cv2.inRange(hsv,lower,upper)

        erode = cv2.erode(thrImg, None, iterations=2)
        dilate = cv2.dilate(erode, None, iterations=2)

        contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.isDetected = False
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w / 2, y + h / 2

            if (abs(h * w)**2) > cv2.getTrackbarPos('Box filter', 'Trackbars'):

                cv2.rectangle(frame, (x, y), (x + w, y + h), [0, 0, 255], 2)
                self.isDetected = True

        if cv2.getTrackbarPos('Caliberate', 'Trackbars') == 1:
            cv2.imshow('Output', thrImg)
        else:
            cv2.imshow('Output', frame)

        try:
            self.isDetected_pub.publish(self.isDetected)
            self.detector_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
            self.hsv_pub.publish(self.bridge.cv2_to_imgmsg(thrImg, "mono8"))

        except CvBridgeError as e:
            print(e)


def main(args):
    print("got to mains###########################################")

    rospy.init_node('gate_detector', anonymous=True)
    gd = gate_detector()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
#cv2.destroyAllWindows()

if __name__ == '__main__':

    main(sys.argv)
