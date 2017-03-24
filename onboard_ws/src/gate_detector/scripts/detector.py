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
from rover_msgs.msg import Drive, GateInfo
# When the rover comes to an estimated GPS location of the gate, a flag will be raised and the gate detector will initialize.
# The gate detector should have a searching behavior that turns the wheels of the rover (or the servos of the zed gimbal) in order to
# find yellow tennis ball. The hsv thresholds can be calibrated using dynamic_reconfigure.
# When the ball is detected, the node will publish a topic to the PID controller which will then hone in on the gate
# to get to the correct distance.

# What this program is currently missing is the search behavior. We will probably use torque commands to search
# for the tennis ball. When the tennis ball is detected, the PID controller will kick in to center the
# tennis ball in the middle of the frame and approach it.


class gate_detector:
    def __init__(self):

        # General use publishers
        self.detector_pub = rospy.Publisher("gate_detector/image_detector", Image, queue_size=10)
        self.hsv_pub = rospy.Publisher("gate_detector/hsv", Image, queue_size=10)
        self.isDetected_pub = rospy.Publisher("gate_detector/isDetected", Bool, queue_size = 1)

        # Publisher of custom message type GateInfo. This will publish to the PID controller
        self.gateInfo_pub = rospy.Publisher("gate_detector/gate_info", GateInfo, queue_size=1)

        # Subscribe to raw images
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)

        # Server to dynamically reconfigure HSV thresholds
        self._server = Server(DetectorConfig, self.reconfigure_callback)

        # CvBridge for message conversion
        self.bridge = CvBridge()

        # Parameters for HSV thresholds
        self._params.hl = rospy.get_param('hue_lower', 30 )
        self._params.hu = rospy.get_param('hue_upper', 36 )
        self._params.sl = rospy.get_param('sat_lower', 82)
        self._params.su = rospy.get_param('sat_upper', 155)
        self._params.vl = rospy.get_param('val_lower', 127)
        self._params.vu = rospy.get_param('val_upper', 255)
        # self._params.implementation = rospy.get_param('implementation', 'square')

        # Instantiation of GateInfo object
        self.gi = GateInfo()
        self.gi.gate_detected = False
        self.gi.image_size = np.array([0 , 0])
        self.gi.box_width = 0
        self.gi.coords = [0 , 0]
        self.gi.gate_distance = 0

        self.gateDetectorEnable = True
        self.implemenation = 'circle'  # can be 'circle' rectangle





    # Clever way to organize parameters for the dynamic reconfigure
    class params_s:
        hl = 0
        hu = 0
        sl = 0
        su = 0
        vl = 0
        vu = 0
        # implemenation = ''

    # I don't think we need this. Already have class variable _params
    _params = params_s()

    # def search_mode(self):

    # callback for the
    def reconfigure_callback(self, config, level):
        print "Reconfigure Callback"

        self._params.hl = config.hue_lower
        self._params.hu = config.hue_upper
        self._params.sl = config.sat_lower
        self._params.su = config.sat_upper
        self._params.vl = config.val_lower
        self._params.vu = config.val_upper
        return config


        # Focal_length = (pixel_width  * distance) / actual_width
        # distance = (actual width * focal_length) / pixel_width
        # pixel_width = right_side - left_side = (x_center + box_width/2) - (x_center - box_width/2)
    def calculate_distance(self, box_center, box_width, actual_width, focal_length):

        pixel_width = (box_center + box_width/2) - (box_center - box_width/2)
        distance = (actual_width * focal_length) / pixel_width

        return distance


    def callback(self, data):

        if self.gateDetectorEnable:
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
            #
            # hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            # thrImg =cv2.inRange(hsv, lower, upper)
            #
            # res = cv2.bitwise_and(frame, frame, mask=thrImg)
            # blur = cv2.medianBlur(res, 2)
            # cv_image = cv2.bitwise_and(cv_image,cv_image, mask=mask)
            blur = cv2.medianBlur(cv_image, 5)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
            thrImg = cv2.inRange(hsv,lower,upper)
            # try hihat function that erodes dilate
            # look up ransac
            # use GPS data to weight images
            # object recognition, machine learning
            # kernel = np.ones((5,5),np.uint8)
            # erode = cv2.erode(thrImg, kernel, iterations=3)
            # dilate = cv2.dilate(erode, kernel, iterations=3)
            erode = cv2.erode(thrImg, None, iterations=3)
            dilate = cv2.dilate(erode, None, iterations=3)

            _,contours,_ = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            self.isDetected = False

            if not contours:
                w,h,x,y = [0,0,0,0]
                self.gi.gate_detected = False

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                # rospy.logwarn(w)
                cx, cy = x + w / 2, y + h / 2

                self.gi.box_width = w
                self.gi.image_size = [frame.shape[0], frame.shape[1]]
                self.gi.coords = [x, y]

                self.gi.gate_distance = self.calculate_distance(x, w, 2.7, 679.05882);


                cv2.rectangle(frame, (x, y), (x + w, y + h), [0, 0, 255], 2)
                self.gi.gate_detected = True
                self.isDetected = True

            calibrate = 1

            # if calibrate == 1:
            #     cv2.imshow('Output', thrImg)
            # else:
            #     cv2.imshow('Output', frame)

            try:
                self.isDetected_pub.publish(self.isDetected)
                self.detector_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
                self.hsv_pub.publish(self.bridge.cv2_to_imgmsg(thrImg, "mono8"))
                self.gateInfo_pub.publish(self.gi)

            except CvBridgeError as e:
                print(e)

        #################################################
        # using circles
        #################################################
        # if self._params.implementation == 'circle':
        #     img_msg = data
        #     try:
        #          cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="rgb8")
        #     except CvBridgeError as e:
        #         print(e)
        #
        #     frame = cv_image
        #     lower =  np.array([self._params.hl, self._params.sl, self._params.vl])
        #     upper =  np.array([self._params.hu, self._params.su, self._params.vu])
        #
        #
        #     blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        #     hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        #
        #
        #     thrImg = cv2.inRange(hsv, lower, upper)
        #     mask = cv2.erode(thrImg, None, iterations=2)
        #     mask =cv2.dilate(mask, None, iterations=2)
        #
        #
        #     cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #     center = None
        #
        #     if len(cnts) > 0:
        #         self.gi.gate_detected = True
        #         c = max(cnts, key=cv2.contourArea)
        #         (x, y), radius =  cv2.minEnclosingCircle(c)
        #         M = cv2.moments(c)
        #         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #
        #         if radius > 1:
        #             cv2.circle(frame, (int(x), int(y)), int(radius),
        #                         (0, 255, 255), 2)
        #             cv2.circle(frame, center, 5, (0,0,255), -1)
        #
        #
        #             self.gi.box_width = radius * 2.0
        #             self.gi.coords = [x, y]
        #             self.gi.image_size = [frame.shape[0], frame.shape[1]]
        #     else :
        #
        #         radius = 0
        #         x = 0
        #         y = 0
        #         self.gi.gate_detected = False
        #
        #     try:
        #         # self.isDetected_pub.publish(self.isDetected)
        #         cv2.imshow('Output', hsv)
        #         self.detector_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        #         self.hsv_pub.publish(self.bridge.cv2_to_imgmsg(thrImg, "mono8"))
        #         self.gateInfo_pub.publish(self.gi)
        #
        #     except CvBridgeError as e:
        #         print(e)
        #
        #
        #
        #
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
