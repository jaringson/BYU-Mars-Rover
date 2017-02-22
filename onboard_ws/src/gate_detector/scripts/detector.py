#!/usr/bin/env python

import roslib
import cv2
import numpy as np
import rospy, sys, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from gate_detector.cfg import DetectorConfig
from cv_bridge import CvBridge, CvBridgeError

class gate_detector:
    def __init__(self):

        self.detector_pub = rospy.Publisher("gate_detector/image_detector", Image, queue_size=10)
        self.hsv_pub = rospy.Publisher("gate_detector/hsv", Image, queue_size=10)
        self.isDetected_pub = rospy.Publisher("gate_detector/isDetected", Bool, queue_size = 1)

        self._server = Server(DetectorConfig, self.reconfigure_callback)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        self._params.hl = rospy.get_param('hue_lower', 28 )
        self._params.hu = rospy.get_param('hue_upper', 54 )
        self._params.sl = rospy.get_param('sat_lower', 136)
        self._params.su = rospy.get_param('sat_upper', 255)
        self._params.vl = rospy.get_param('val_lower', 111)
        self._params.vu = rospy.get_param('val_upper', 255)
        self.isDetected = False

    class params_s:
        hl = 0
        hu = 0
        sl = 0
        su = 0
        vl = 0
        vu = 0

    _params = params_s()

    def reconfigure_callback(self, config, level):
        print "Reconfigure Callback"

        self._params.hl = config.hue_lower
        self._params.hu = config.hue_upper
        self._params.sl = config.sat_lower
        self._params.su = config.sat_upper
        self._params.vl = config.val_lower
        self._params.vu = config.val_upper
        return config

    def callback(self, data):


        img_msg = data

        try:

            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            print(e)

        # cv_image = cv2.cvtColor(cv_image, cv2.)
        # this command flips the frame around the y axis

        frame = cv_image
        lower =  np.array([self._params.hl, self._params.sl, self._params.vl])
        upper =  np.array([self._params.hu, self._params.su, self._params.vu])
        blur = cv2.medianBlur(cv_image, 3)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        thrImg = cv2.inRange(hsv,lower,upper)
        # try hihat function that erodes dilate
        # look up ransac
        # use GPS data to weight images
        # object recognition, machine learning

        erode = cv2.erode(thrImg, None, iterations=2)
        dilate = cv2.dilate(erode, None, iterations=2)

        _,contours,_ = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.isDetected = False
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w / 2, y + h / 2

  #         if (abs(h * w)**2) > cv2.getTrackbarPos('Box filter', 'Trackbars'):

            cv2.rectangle(frame, (x, y), (x + w, y + h), [0, 0, 255], 2)
            self.isDetected = True

        # if cv2.getTrackbarPos('Calibrate', 'Trackbars') == 0:
        #     cv2.imshow('Output', thrImg)
        # else:
        #     cv2.imshow('Output', frame)

        calibrate = 1

        if calibrate == 1:
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
    r = rospy.Rate(15)
    try:
        rospy.spin()
        r.sleep()
    except KeyBoardInterrupt:
        print("Shutting down")
#cv2.destroyAllWindows()

if __name__ == '__main__':

    main(sys.argv)
