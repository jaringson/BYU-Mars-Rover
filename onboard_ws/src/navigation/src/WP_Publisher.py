#!/usr/bin/env python

import rospy
from Supervisor import Supervisor
from rover_msgs.srv import WaypointSend

if __name__ == '__main__':
    rospy.wait_for_service('NewWaypoints')
    WPsrv = rospy.ServiceProxy('NewWaypoints', WaypointSend)
    wp_x = [10,10,-25,0]
    wp_y = [5,-30,-40,0]
    status = WPsrv(wp_x,wp_y,'tester')
    print status
    rospy.loginfo('Waypoints Received')
    while not rospy.is_shutdown():
        a = 1

