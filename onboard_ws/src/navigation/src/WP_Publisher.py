#!/usr/bin/env python

import rospy
from Supervisor import Supervisor
from rover_msgs.srv import WaypointSend

if __name__ == '__main__':
    rospy.wait_for_service('NewWaypoints')
    WPsrv = rospy.ServiceProxy('NewWaypoints', WaypointSend)
    wp_x = [0,1]
    wp_y = [1,2]
    status = WPsrv(wp_x,wp_y,'tester')
    print status
    rospy.loginfo('Waypoints Received')
    while not rospy.is_shutdown():
        a = 1

