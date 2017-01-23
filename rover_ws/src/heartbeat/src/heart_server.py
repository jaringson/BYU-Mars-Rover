#!/usr/bin/env python

from heartbeat.srv import *
import rospy

def handle_heartbeat(req):
    # print "Returning [%s, %s]"%(req.send, req.receive)
    return HeartbeatResponse(True)

def heartbeat_server():
    rospy.init_node('heartbeat_server')
    s = rospy.Service('heartbeat', Heartbeat, handle_heartbeat)
    print "Ready to heartbeat"
    rospy.spin()

if __name__ == "__main__":
    heartbeat_server()