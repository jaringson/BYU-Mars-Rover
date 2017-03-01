#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import param as P
from rover_msgs.msg import GateInfo
from rover_msgs.msg import Drive
from std_msgs.msg import Int16

import controllerPID as ctrl

# When a new message is sent to the topic '/gate_info',
# this function will run.
def callback(data):
    #only do stuff if gate is detected

    if (data.gate_detected):

        '''
        need another logic statement for if error_d1>something
        if 1:
            do all the turning stuff (xcontrol)
        else:
            go to the gate (zcontrol, based on box size)
        '''

        #message contains coordinates of ball centroid and image dimensions
        x = data.coords[0]
        image_width = data.image_size[1]

        #states are [x1-x2,(x1-x2)dot].T, we care about x1-x2
        states = x-(image_width-x)
        print ('states: ' , states)

        # the desired reference will always be 0 because we want to center the ball
        ref_input = 0

        u = control.getInputs(ref_input,states) # Calculate the forces

        if (states < 100 and states > -100):
             command.lw = abs(u)
             command.rw = abs(u)
             command_pub.publish(command)
             right.publish(command.rw)
             left.publish(command.lw)
        else:
            command.lw = -u
            command.rw = u
            command_pub.publish(command)
            right.publish(command.rw)
            left.publish(command.lw)


if __name__ == '__main__':

    control = ctrl.controllerPID()                         # Instantiate controllerPD class
    i=0
    # Create the node
    rospy.init_node('gate_honing_controller', anonymous=False)

    # Subscriber to topic '/gate_info' which contains the pixel location of the
    # gate, (along with size of the box?)
    rospy.Subscriber('gate_detector/gate_info', GateInfo, callback)

    # Publisher to topic 'command'
    #-----PUBLISH TO ROVER COMMAND
    command_pub = rospy.Publisher('SOMETHING',Drive,queue_size=5)
    right = rospy.Publisher('/right',Int16,queue_size=5)
    left = rospy.Publisher('/left',Int16,queue_size=5)
    # Command Message object
    command=Drive()


    try:

        # Keep node alive until ROS is shutdown.
        while not rospy.is_shutdown():
            rospy.spin()

        # The ROSInterruptException is raised if the program is killed
        # while sleeping with rospy.sleep() or rospy.rate.sleep()
    except rospy.ROSInterruptException:
        pass
