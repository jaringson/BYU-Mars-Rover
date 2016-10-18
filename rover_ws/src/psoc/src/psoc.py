#!/usr/bin/env python

import serial, rospy, struct
from rover_msgs.msg import All, Pololu, SciFeedback
from std_msgs.msg import ByteMultiArray
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


        # feedback: 0xE3, turret-low, turret-high, shoulder-low, shoulder-high,
        # elbow-low, elbow-high, forearm-low, forearm-high, temperature-low,
        # temperature-high, humidity-low, humidity-high
        self.feedback = ByteMultiArray()
        self.science_data = SciFeedback()
        self.arm_feedback = Pololu()

        # initialize serial port here
        self.ser = serial.Serial('/dev/ttyUSB2', 57600, timeout = 1)
	# if self.ser.is_open():
	# 	self.ser.close()

        # initialize subscribers
        self.sub_cmd = rospy.Subscriber('rover_command', All, self.set_rover_cmd)
        self.sub_joint = rospy.Subscriber('/joint_cmd', sensor_msgs/JointState, se
        
        # initialize publishers
        self.pub_sci = rospy.Publisher('/science_feedback', SciFeedback, queue_size=1)
        self.pub_arm = rospy.Publisher('/arm_feedback', Pololu, queue_size=1)

    # Callback
    def set_rover_cmd(self, cmd):
        self.msg.data[0] = 0xEA
        self.msg.data[1] = cmd.lw & 0xff
        self.msg.data[2] = (cmd.lw & 0xff00) >> 8
        self.msg.data[3] = cmd.rw & 0xff
        self.msg.data[4] = (cmd.rw & 0xff00) >> 8
        self.msg.data[5] = cmd.pan & 0xff
        self.msg.data[6] = (cmd.pan & 0xff00) >> 8
        self.msg.data[7] = cmd.tilt & 0xff
        self.msg.data[8] = (cmd.tilt & 0xff00) >> 8
        self.msg.data[9] = cmd.camnum
        self.msg.data[10] = cmd.q1 & 0xff
        self.msg.data[11] = (cmd.q1 & 0xff00) >> 8
        self.msg.data[12] = cmd.q2 & 0xff
        self.msg.data[13] = (cmd.q2 & 0xff00) >> 8
        self.msg.data[14] = cmd.q3 & 0xff
        self.msg.data[15] = (cmd.q3 & 0xff00) >> 8
        self.msg.data[16] = cmd.q4 & 0xff
        self.msg.data[17] = (cmd.q4 & 0xff00) >> 8
        self.msg.data[18] = cmd.q5 & 0xff
        self.msg.data[19] = 0
        self.msg.data[20] = 0
        self.msg.data[21] = 0
        self.msg.data[22] = cmd.grip & 0xff
        self.msg.data[23] = (cmd.grip & 0xff00) >> 8
        self.msg.data[24] = cmd.chutes
        self.msg.data[25] = cmd.shovel & 0xff
        self.msg.data[26] = (cmd.shovel & 0xff00) >> 8

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
		    self.pub_arm.publish(self.arm_feedback)
		    self.pub_sci.publish(self.science_data)


if __name__ == '__main__':
    rospy.init_node('psoc_node', anonymous=True)
    hz = 60.0
    rate = rospy.Rate(hz)
    # call the constructor
    psoc = PSOC()

    while not rospy.is_shutdown():
        psoc.read_feedback()
        rate.sleep()

    psoc.ser.close()
