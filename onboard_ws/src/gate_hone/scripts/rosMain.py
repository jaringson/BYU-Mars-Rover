#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import param as P
from rover_msgs.msg import GateCoord
from rover_msgs.msg import Drive

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

        # Calculate time elapsed since last function call, s
        global prev_time
        now = rospy.Time.now()
        dt = (now-prev_time).to_sec()
        prev_time = now               # Update prev_time , time
        global sim_time
        sim_time = round(sim_time+dt,6)

        # the desired reference will always be 0 because we want to center the ball
        ref_input = 0

        u = ctrl.getInputs(ref_input,states) # Calculate the forces

        command.lw = -u
        command.rw = u
        command_pub.publish(command)

if __name__ == '__main__':

    ctrl = ctrl()                         # Instantiate controllerPD class

    # Create the node
    rospy.init_node('gate_honing_controller', anonymous=False)

    # Subscriber to topic '/gate_info' which contains the pixel location of the
    # gate, (along with size of the box?)
    rospy.Subscriber('gate_detector/gate_info', GateInfo, callback)

    # Publisher to topic 'command'
    #-----PUBLISH TO ROVER COMMAND
    command_pub = rospy.Publisher('SOMETHING',Drive,queue_size=5)

    # Command Message object
    command=Drive()

    # Used to calculate the time between the callback function calls.
    global prev_time
    prev_time= rospy.Time.now()            # Gets the current time, time
    global sim_time
    sim_time = 0

    try:

        # Keep node alive until ROS is shutdown.
        while not rospy.is_shutdown():
            rospy.spin()

        # The ROSInterruptException is raised if the program is killed
        # while sleeping with rospy.sleep() or rospy.rate.sleep()
    except rospy.ROSInterruptException:
        pass
