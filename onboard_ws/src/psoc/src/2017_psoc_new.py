#!/usr/bin/env python

import serial, rospy, struct
from rover_msgs.msg import Drive, RoverState, PSOC17, Science # Pololu, SciFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray, Int8
import re
import numpy as np

class PSOC_class():

	def __init__(self):
		# 2017 Psoc Code
		# Premble (1 Byte) (0)
		# LW Speed (2 Bytes) (1,2)
		# LW Direction (1 Byte) (3)
		# RW Speed (2 Bytes) (4,5)
		# RW Direction (1 Byte) (6)
		# Turret (2 Bytes) (7,8)
		# Shoulder/Elevator (2 Bytes) (9, 10)
		# Elbow/Plunge (2 Bytes) (11,12)
		# Forearm/Drill (2 Bytes) (13,14)
		# Plate Shift (2 Bytes) (15,16)
		# Hand (1 Byte) (17) # 0 - nothing, 1 2
		# Chutes (1 Byte) (18)
		# Total = 19 Bytes
		#
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

		# Init PSOC Message
		self.psoc = PSOC17()

		# initialize Drive msg stuff
		self.psoc.lw = np.uint16(0)
		self.psoc.rw = np.uint16(0)
		self.psoc.lwdirection = 1 # 1 if forward, 0 is backward
		self.psoc.rwdirection = 1 # 1 if forward, 0 if backward

		# initialize chutes stuff
		self.psoc.chutes = np.uint16(0)

		# initialize JointState Stuff
		self.psoc.q1 = np.uint16(2046)
		self.psoc.q2 = np.uint16(2046)
		self.psoc.q3 = np.uint16(2046)
		self.psoc.q4 = np.uint16(2046)
		self.psoc.q5 = np.uint16(2046)
		self.psoc.q6 = np.uint16(2046)
		self.psoc.plate = np.uint16(0)

		# initialize Grip Stuff
		self.psoc.grip = np.uint16(0)

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
		self.sub_drive = rospy.Subscriber('/drive_cmd', Drive, self.drive_callback)
		self.sub_state = rospy.Subscriber('/rover_state_cmd', RoverState, self.state_callback)
		self.sub_joint = rospy.Subscriber('/joint_cmd', JointState, self.joint_callback)
		self.sub_science = rospy.Subscriber('/science_cmd', Science, self.science_callback)
		self.sub_grip = rospy.Subscriber('/grip', Int8, self.grip_callback)        

        # initialize publishers
		self.pub_psoc = rospy.Publisher('/psoc_out', PSOC17, queue_size=1)
		print "Successfull"
     
   #self.pub_arm = rospy.Publisher('/arm_feedback', Pololu, queue_size=1)

    # Callback
    
    # Drive callback

	def drive_callback(self, drive):
    # Lastyear .lw & .rw were from 1000 to 2000, 1500 is no movement
    # This year .l1 & .rw are from -100 to 100
    # Map 0-100 speed to 0-20,000
    # Map +/- to .lwdirection as 0 for forward, 1 for backward
		
		self.psoc.lw = np.abs(drive.lw)*200
		self.psoc.rw = np.abs(drive.rw)*200

		if np.sign(drive.lw) == -1:
			self.psoc.lwdirection = 1
		else:
			self.psoc.lwdirection = 0

		if np.sign(drive.rw) == -1:
			self.psoc.rwdirection = 1
		else:
			self.psoc.rwdirection = 0

		self.set_rover_cmd()

    # Rover State Callback
    #     
	def state_callback(self, state):
		
		# self.psoc.camnum = np.uint16(state.camnum)
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

    # Science Callback
	def science_callback(self, msg):

		# Science Uses the same pololus as the Arm:
		# Here are which pololus correspond to which
		# q1 = 0 # not used for science
		# q2 = elevator = Shoulder
		# q3 = Plunge = Elbow
		# q4 = Drill = Forearm
		# plate = Science Plate Shift

		self.psoc.q2 = np.uint16(msg.elevator)
		self.psoc.q3 = np.uint16(msg.plunge)

		# CHECK WHAT TO SEND FOR THE DRILL WHEN ON/OFF
		if msg.drill:
			self.psoc.q4 = np.uint16(4095)
		elif not msg.drill:
			self.psoc.q4 = np.uint16(2048)

		self.psoc.plate = np.uint16(msg.plate)

		self.psoc.q1 = np.uint16(0)
		self.psoc.q5 = np.uint16(0)
		self.psoc.q6 = np.uint16(0)

		self.set_rover_cmd()

	def grip_callback(self, grip):
		self.psoc.grip = np.uint8(grip.data)

		self.set_rover_cmd()

		#print "grip"
		#print grip.data
		#print type(grip.data)
		#print np.uint16(grip.data)
    
	def set_rover_cmd(self):

		# 2017 Psoc Code
		# Premble (1 Byte) (0)
		# LW Speed (2 Bytes) (1,2)
		# LW Direction (1 Byte) (3)
		# RW Speed (2 Bytes) (4,5)
		# RW Direction (1 Byte) (6)
		# Turret (2 Bytes) (7,8)
		# Shoulder/Elevator (2 Bytes) (9, 10)
		# Elbow/Plunge (2 Bytes) (11,12)
		# Forearm/Drill (2 Bytes) (13,14)
		# Plate Shift (2 Bytes) (15,16)
		# Hand (1 Byte) (17) # 0 - nothing, 1 2
		# Chutes (1 Byte) (18)
		# Total = 19 Bytes

		self.msg.data[0] = 0xEA
		self.msg.data[1] = self.psoc.lw & 0xff
		self.msg.data[2] = (self.psoc.lw & 0xff00) >> 8
		self.msg.data[3] = self.psoc.lwdirection # because only 0 or 1

		self.msg.data[4] = self.psoc.rw & 0xff
		self.msg.data[5] = (self.psoc.rw & 0xff00) >> 8
		self.msg.data[6] = self.psoc.rwdirection # because only 0 or 1

		self.msg.data[7] = self.psoc.q1 & 0xff
		self.msg.data[8] = (self.psoc.q1 & 0xff00) >> 8
		self.msg.data[9] = self.psoc.q2 & 0xff
		self.msg.data[10] = (self.psoc.q2 & 0xff00) >> 8
		self.msg.data[11] = self.psoc.q3 & 0xff
		self.msg.data[12] = (self.psoc.q3 & 0xff00) >> 8
		self.msg.data[13] = self.psoc.q4 & 0xff
		self.msg.data[14] = (self.psoc.q4 & 0xff00) >> 8
		self.msg.data[15] = self.psoc.plate & 0xff
		self.msg.data[16] = (self.psoc.plate & 0xff00) >> 8

		self.msg.data[17] = self.psoc.grip
		self.msg.data[18] = self.psoc.chutes

		# print 'Ser open?'
		# print self.ser.isOpen()

		string = ''
		for i in self.msg.data:
			string += struct.pack('!B',i)
		bwrite = self.ser.write(string)
		# print bwrite

		# publish values just written to psoc
		self.pub_psoc.publish(self.psoc)

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
