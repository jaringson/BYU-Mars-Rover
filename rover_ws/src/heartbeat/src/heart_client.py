#!/usr/bin/env python

import rospy
from heartbeat.srv import *
from std_msgs.msg import String

def return_beat():
    try:
        rospy.wait_for_service('heartbeat', timeout=1)
    except rospy.ROSException:
        return False
    try:
        heart_service = rospy.ServiceProxy('heartbeat', Heartbeat, persistent=True)
        resp1 = heart_service(True)
        return resp1.receive
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    pub = rospy.Publisher('heartbeat_flag', String, queue_size=1)
    rospy.init_node('heart', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    comsloss_time = 0;

    while not rospy.is_shutdown():
        answer = return_beat()
        # print "Heartbeat returned"
        
        if not answer:
            if comsloss_time == 0:
                comsloss_time = 2
            else:
                comsloss_time +=1
        else:
            comsloss_time = 0
        print comsloss_time

        if answer:
            pub.publish('Comms Good')
        elif (not answer) and (comsloss_time >= 180):
            pub.publish('Comms Lost Greater than 3 Mins')
        elif (not answer) and (comsloss_time >= 30):
            pub.publish('Comms Lost Greater than 30 Sec')
        elif (not answer):
            pub.publish('Comms Lost Less than 30 Sec')

        rate.sleep()
