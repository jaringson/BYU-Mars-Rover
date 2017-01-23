#!/usr/bin/env python

import serial, rospy, struct
from rover_msgs.msg import Drive, RoverState, PSOC # Pololu, SciFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray, Int8
import re
import numpy as np

class PSOC_class():

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

		# Init PSOC Message
		self.psoc = PSOC()

		# initialize Drive msg stuff
		self.psoc.lw = np.uint16(1500)
		self.psoc.rw = np.uint16(1500)

		# initialize RoverState stuff
		self.psoc.pan = np.uint16(1500)
		self.psoc.tilt = np.uint16(1500)
		self.psoc.camnum = np.uint16(0)
		self.psoc.chutes = np.uint16(0)

		# initialize JointState Stuff
		self.psoc.q1 = np.uint16(2046)
		self.psoc.q2 = np.uint16(2046)
		self.psoc.q3 = np.uint16(2046)
		self.psoc.q4 = np.uint16(2046)
		self.psoc.q5 = np.uint16(2046)
		self.psoc.q6 = np.uint16(2046)

		# initialize Grip Stuff
		self.psoc.grip = np.uint16(0)

		# feedback: 0xE3, turret-low, turret-high, shoulder-low, shoulder-high,
		# elbow-low, elbow-high, forearm-low, forearm-high, temperature-low,
		# temperature-high, humidity-low, humidity-high
		self.feedback = ByteMultiArray()
		#self.science_data = SciFeedback()
		#self.arm_feedback = Pololu()

		# initialize serial port here
		self.ser = serial.Serial('/dev/ttyUSB3', 57600, timeout = 1)
		# if self.ser.is_open():
		# 	self.ser.close()

		# initialize subscribers
		self.sub_drive = rospy.Subscriber('/drive_cmd', Drive, self.drive_callback)
		self.sub_state = rospy.Subscriber('/rover_state_cmd', RoverState, self.state_callback)
		self.sub_joint = rospy.Subscriber('/joint_cmd', JointState, self.joint_callback)
		self.sub_grip = rospy.Subscriber('/grip', Int8, self.grip_callback)        

        # initialize publishers
		self.pub_psoc = rospy.Publisher('/psoc_out', PSOC, queue_size=1)
        #self.pub_arm = rospy.Publisher('/arm_feedback', Pololu, queue_size=1)

    # Callback
    
    # Drive callback

	def drive_callback(self, drive):
    # Lastyear .lw & .rw were from 1000 to 2000, 1500 is no movement
    # This year .l1 & .rw are from -100 to 100

		lw_temp = 1500 + (5*drive.lw)
		rw_temp = 1500 + (5*-drive.rw)

		self.psoc.lw = np.uint16(lw_temp)
		self.psoc.rw = np.uint16(rw_temp)

		self.set_rover_cmd()

    # Rover State Callback
    #     
	def state_callback(self, state):
		# self.state.pan = state.pan
		# self.state.tilt = state.tilt 
		self.psoc.pan = np.uint16(1500)
		self.psoc.tilt = np.uint16(1500) 
		self.psoc.camnum = np.uint16(state.camnum)
		self.psoc.chutes = np.uint16(state.chutes)

		self.set_rover_cmd()

    # Joint Callback
    # Last year = values from 0 to 4092 for each joint, representing commanded angle
    # Giving radian angle for each joint (we can give you velocities scalars from -100 to 100 if you want)
	def joint_callback(self, joint):
		pos_temp = [0, 0, 0, 0, 0, 0]

		# assume all joint go from -pi to pi
		# 0 = -pi; 4092 = pi
		pos_temp[0] = 2046 + 2046*(joint.position[0]/np.pi)
		pos_temp[1] = 2046 + 2046*(joint.position[1]/np.pi)
		pos_temp[2] = 2046 + 2046*(joint.position[2]/np.pi)
		pos_temp[3] = 2046 + 2046*(joint.position[3]/np.pi)
		pos_temp[4] = 2046 + 2046*(joint.position[4]/np.pi)
		pos_temp[5] = 2046 + 2046*(joint.position[5]/np.pi)

		self.psoc.q1 = np.uint16(pos_temp[0])
		self.psoc.q2 = np.uint16(pos_temp[1])
		self.psoc.q3 = np.uint16(pos_temp[2])
		self.psoc.q4 = np.uint16(pos_temp[3])
		self.psoc.q5 = np.uint16(pos_temp[4])
		self.psoc.q6 = np.uint16(pos_temp[5])

		self.set_rover_cmd()

		# self.msg.data[0] = 0xEA
		# self.msg.data[1] = self.psoc.lw & 0xff
		# self.msg.data[2] = (self.psoc.lw & 0xff00) >> 8
		# self.msg.data[3] = self.psoc.rw & 0xff
		# self.msg.data[4] = (self.psoc.rw & 0xff00) >> 8
		# self.msg.data[5] = self.psoc.pan & 0xff
		# self.msg.data[6] = (self.psoc.pan & 0xff00) >> 8
		# self.msg.data[7] = self.psoc.tilt & 0xff
		# self.msg.data[8] = (self.psoc.tilt & 0xff00) >> 8
		# self.msg.data[9] = self.psoc.camnum
		# self.msg.data[10] = self.psoc.q1 & 0xff
		# self.msg.data[11] = (self.psoc.q1 & 0xff00) >> 8
		# self.msg.data[12] = self.psoc.q2 & 0xff
		# self.msg.data[13] = (self.psoc.q2 & 0xff00) >> 8
		# self.msg.data[14] = self.psoc.q3 & 0xff
		# self.msg.data[15] = (self.psoc.q3 & 0xff00) >> 8
		# self.msg.data[16] = self.psoc.q4 & 0xff
		# self.msg.data[17] = (self.psoc.q4 & 0xff00) >> 8
		# self.msg.data[18] = self.psoc.q5 & 0xff
		# self.msg.data[19] = 0
		# self.msg.data[20] = 0
		# self.msg.data[21] = 0
		# self.msg.data[22] = self.psoc.grip & 0xff
		# self.msg.data[23] = (self.psoc.grip & 0xff00) >> 8
		# self.msg.data[24] = self.psoc.chutes
		# self.msg.data[25] = np.uint16(1500) & 0xff #cmd.psoc.shovel & 0xff
		# self.msg.data[26] = (np.uint16(1500) & 0xff00) >> 8#(cmd.psoc.shovel & 0xff00) >> 8

		# # print 'Ser open?'
		# # print self.ser.isOpen()

		# string = ''
		# for i in self.msg.data:
		# 	string += struct.pack('!B',i)
		# bwrite = self.ser.write(string)

		# rospy.logwarn(str(pos_temp))

	def grip_callback(self, grip):
		self.psoc.grip = np.uint16(grip)

		self.set_rover_cmd()
    
	def set_rover_cmd(self):

		# # Test Prints
		# print 'Drive lw/rw'
		# print self.psoc.lw
		# print self.psoc.rw

		# print 'Pan/Tilt'
		# print self.psoc.pan
		# print self.psoc.tilt

		# print 'Joints'
		# print self.psoc.q1
		# print self.psoc.q2
		# print self.psoc.q3
		# print self.psoc.q4

		self.msg.data[0] = 0xEA
		self.msg.data[1] = self.psoc.lw & 0xff
		self.msg.data[2] = (self.psoc.lw & 0xff00) >> 8
		self.msg.data[3] = self.psoc.rw & 0xff
		self.msg.data[4] = (self.psoc.rw & 0xff00) >> 8
		self.msg.data[5] = self.psoc.pan & 0xff
		self.msg.data[6] = (self.psoc.pan & 0xff00) >> 8
		self.msg.data[7] = self.psoc.tilt & 0xff
		self.msg.data[8] = (self.psoc.tilt & 0xff00) >> 8
		self.msg.data[9] = self.psoc.camnum
		self.msg.data[10] = self.psoc.q1 & 0xff
		self.msg.data[11] = (self.psoc.q1 & 0xff00) >> 8
		self.msg.data[12] = self.psoc.q2 & 0xff
		self.msg.data[13] = (self.psoc.q2 & 0xff00) >> 8
		self.msg.data[14] = self.psoc.q3 & 0xff
		self.msg.data[15] = (self.psoc.q3 & 0xff00) >> 8
		self.msg.data[16] = self.psoc.q4 & 0xff
		self.msg.data[17] = (self.psoc.q4 & 0xff00) >> 8
		self.msg.data[18] = self.psoc.q5 & 0xff
		self.msg.data[19] = 0
		self.msg.data[20] = 0
		self.msg.data[21] = 0
		self.msg.data[22] = self.psoc.grip & 0xff
		self.msg.data[23] = (self.psoc.grip & 0xff00) >> 8
		self.msg.data[24] = self.psoc.chutes
		self.msg.data[25] = np.uint16(1500) & 0xff #cmd.psoc.shovel & 0xff
		self.msg.data[26] = (np.uint16(1500) & 0xff00) >> 8#(cmd.psoc.shovel & 0xff00) >> 8

		# print 'Ser open?'
		# print self.ser.isOpen()

		string = ''
		for i in self.msg.data:
			string += struct.pack('!B',i)
		bwrite = self.ser.write(string)
#		print bwrite

		# publish values just written to psoc
		# self.pub_psoc.publish(self.psoc)

	def remove_non_ascii(self,text):
		return ''.join([i if ord(i)<128 else ' ' for i in text])

	def read_feedback(self):
		pass
		# # read feedback from psoc here if at least 13 bytes have been received
		# if self.ser.inWaiting() >= 13:
		# 	while self.ser.inWaiting() > 0:
		# 		if self.ser.read(1).encode('hex') in 'e3':
		# 			self.arm_feedback.q1 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			self.arm_feedback.q2 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			self.arm_feedback.q3 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			self.arm_feedback.q4 = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			self.science_data.temp = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			self.science_data.humidity = ord(self.ser.read(1).decode('string_escape')) | (ord(self.ser.read(1).decode('string_escape')) << 8)
		# 			#		    self.pub_arm.publish(self.arm_feedback)
		# 			#		    self.pub_sci.publish(self.science_data)


if __name__ == '__main__':
	rospy.init_node('psoc_node', anonymous=True)
	hz = 60.0
	rate = rospy.Rate(hz)
	# call the constructor
	psoc = PSOC_class()

	while not rospy.is_shutdown():
		# psoc.set_rover_cmd()
		# psoc.read_feedback()
		rate.sleep()

	psoc.ser.close()
