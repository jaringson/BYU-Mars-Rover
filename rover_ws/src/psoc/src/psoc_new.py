#!/usr/bin/env python

import serial, rospy, struct
from rover_msgs.msg import Drive, RoverState # Pololu, SciFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray, Int8
import re

class PSOC():

    def __init__(self):
        # message:
        # 0xEA, left-wheel-low, left-wheel-high, right-wheel-low,
        #   right-wheel-high,
        # cam-pan-low, cam-pan-high, cam-tilt-low, cam-tilt-high, cam-sel,
        # turret-low, turret-high, shoulder-low, shoulder-high, elbow-low,
        # elbow-high, forearm-low, forearm-high, 0x0, 0x0, 0x0, 0x0, hand-low,
        # hand-high, chutes, shovel-low, shovel-high
        #
        # total number of bytes = 27
        self.msg = ByteMultiArray()
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        self.msg.data.append(0)
        
        # initialize Drive msg stuff
        self.drive.lw = 0
        self.drive.rw = 0
        
        # initialize RoverState stuff
        self.state.pan = 0
        self.state.tilt = 0
        self.state.camnum = 0
        self.state.chutes = 0
        
        # initialize JointState Stuff
        self.joint.q1 = 0.0
        self.joint.q2 = 0.0
        self.joint.q3 = 0.0
        self.joint.q4 = 0.0
        self.joint.q5 = 0.0
        self.joint.q6 = 0.0
        
        # initialize Grip Stuff
        self.grip = 0

        # feedback: 0xE3, turret-low, turret-high, shoulder-low, shoulder-high,
        # elbow-low, elbow-high, forearm-low, forearm-high, temperature-low,
        # temperature-high, humidity-low, humidity-high
        self.feedback = ByteMultiArray()
        #self.science_data = SciFeedback()
        #self.arm_feedback = Pololu()

        # initialize serial port here
        self.ser = serial.Serial('/dev/ttyUSB2', 57600, timeout = 1)
	# if self.ser.is_open():
	# 	self.ser.close()

        # initialize subscribers
        self.sub_drive = rospy.Subscriber('/drive_state', rover_msgs/Drive, self.drive_callback)
        self.sub_state = rospy.Subscriber('/rover_state_cmd', rover_msgs/RoverState, self.state_callback)
        self.sub_joint = rospy.Subscriber('/joint_cmd', sensor_msgs/JointState, self.joint_callback)
        self.sub_joint = rospy.Subscriber('/grip', Int8, self.grip_callback)        
        
        # initialize publishers
        #self.pub_sci = rospy.Publisher('/science_feedback', SciFeedback, queue_size=1)
        #self.pub_arm = rospy.Publisher('/arm_feedback', Pololu, queue_size=1)

    # Callback
    
    # Drive callback
    # Lastyear .lw & .rw were from 1000 to 2000
    # This year .l1 & .rw are from -100 to 100
    def drive_callback(self, drive):
        self.drive.lw = drive.lw
        self.drive.rw = drive.rw
    
    # Rover State Callback
    #     
    def state_callback(self, state):
        self.state.pan = state.pan
        self.state.tilt = state.tilt 
        self.state.camnum = state.camnum
        self.state.chutes = state.chutes

    # Joint Callback
    # Check what they were giving last year
    # Giving radian angle for each joint (we can give you velocities scalars from -100 to 100 if you want)
    def joint_callback(self, joint):
        self.joint.q1 = joint.position[0]
        self.joint.q2 = joint.position[1]
        self.joint.q3 = joint.position[2]
        self.joint.q4 = joint.position[3]
        self.joint.q5 = joint.position[4]
        self.joint.q6 = joint.position[5]

    def grip_callback(self, grip)
        self.grip = grip
    
    def set_rover_cmd(self):
        self.msg.data[0] = 0xEA
        self.msg.data[1] = self.drive.lw & 0xff
        self.msg.data[2] = (self.drive.lw & 0xff00) >> 8
        self.msg.data[3] = self.drive.rw & 0xff
        self.msg.data[4] = (self.drive.rw & 0xff00) >> 8
        self.msg.data[5] = self.state.pan & 0xff
        self.msg.data[6] = (self.state.pan & 0xff00) >> 8
        self.msg.data[7] = self.state.tilt & 0xff
        self.msg.data[8] = (self.state.tilt & 0xff00) >> 8
        self.msg.data[9] = 0 #self.state.camnum
        self.msg.data[10] = self.joint.q1 & 0xff
        self.msg.data[11] = (self.joint.q1 & 0xff00) >> 8
        self.msg.data[12] = self.joint.q2 & 0xff
        self.msg.data[13] = (self.joint.q2 & 0xff00) >> 8
        self.msg.data[14] = self.joint.q3 & 0xff
        self.msg.data[15] = (self.joint.q3 & 0xff00) >> 8
        self.msg.data[16] = self.joint.q4 & 0xff
        self.msg.data[17] = (self.joint.q4 & 0xff00) >> 8
        self.msg.data[18] = 0 #cmd.q5 & 0xff
        self.msg.data[19] = 0
        self.msg.data[20] = 0
        self.msg.data[21] = 0
        self.msg.data[22] = 0 #self.grip & 0xff
        self.msg.data[23] = 0 #(self.grip & 0xff00) >> 8
        self.msg.data[24] = 0 #self.state.chutes
        self.msg.data[25] = 0 #cmd.shovel & 0xff
        self.msg.data[26] = 0 #(cmd.shovel & 0xff00) >> 8

	    string = ''
	    for i in self.msg.data:
	        string += struct.pack('!B',i)
            self.ser.write(string)

    def remove_non_ascii(self,text):
        return ''.join([i if ord(i)<128 else ' ' for i in text])

    def read_feedback(self):
        # read feedback from psoc here if at least 13 bytes have been received
        if self.ser.inWaiting() >= 13:
	    while self.ser.inWaiting() > 0:
	        if self.ser.read(1).encode('hex') in 'e3':
	            self.arm_feedback.q1 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
	            self.arm_feedback.q2 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
	            self.arm_feedback.q3 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
	            self.arm_feedback.q4 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
	            self.science_data.temp = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
	            self.science_data.humidity = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
#		    self.pub_arm.publish(self.arm_feedback)
#		    self.pub_sci.publish(self.science_data)


if __name__ == '__main__':
    rospy.init_node('psoc_node', anonymous=True)
    hz = 60.0
    rate = rospy.Rate(hz)
    # call the constructor
    psoc = PSOC()

    while not rospy.is_shutdown():
        psoc.set_rover_cmd()
        psoc.read_feedback()
        rate.sleep()

    psoc.ser.close()
