#!/usr/bin/env python

import rospy
import numpy as np
from Supervisor import Supervisor
from rover_msgs.srv import WaypointSend
from rover_msgs.msg import NavState
from math import *


class WP_Publisher:
    def __init__(self):
        self.sub_navstate = rospy.Subscriber('/estimate', NavState, self.estimateCallback)

        rospy.wait_for_service('NewWaypoints')
        self.WPsrv = rospy.ServiceProxy('NewWaypoints', WaypointSend)
        self.estimate = NavState()


    def estimateCallback(self,msg):
        self.estimate = msg

    def convertAndSendWaypoints(self, wp_lat, wp_long):
        estimate.position[0] = EARTH_RADIUS*(msg.latitude - self.estimate.base_latitude)*np.pi/180.0 #pn
        estimate.position[1] = EARTH_RADIUS*cos(self.estimate.base_latitude*np.pi/180.0)*(msg.longitude - self.estimate.base_longitude)*np.pi/180.0 # lon
        estimate.position[2] = msg.pose.pose.position.z # alt


        EARTH_RADIUS = 6378145.0
        wp_x = []
        wp_y = []
        for i in range(0,len(self.wp_lat)):
            wp_x.append(EARTH_RADIUS*(self.wp_lat[i] - self.estimate.base_latitude)*np.pi/180.0)
            wp_y.append(EARTH_RADIUS*cos(self.estimate.base_latitude*np.pi/180.0) * (self.wp_long[i] - self.estimate.base_longitude)*np.pi/180.0)

        status = WPsrv(wp_x,wp_y,'tester')
        if status:
            rospy.loginfo('Waypoints Received')

if __name__ == '__main__':
    rospy.init_node('wp_publisher')
    rate = rospy.Rate(10)
    
    wp = WP_Publisher()

    while not rospy.is_shutdown():
        rospy.spin()

