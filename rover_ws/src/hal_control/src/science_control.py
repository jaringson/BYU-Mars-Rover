#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
from rover_msgs.msg import ArmState, Science
from sensor_msgs.msg import Joy, JointState 
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
import time
import numpy as np
from urdf_parser_py.urdf import URDF
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
import random
import tf
import tf.transformations as tr

class Arm_XBOX():
	def __init__(self):
	# Variables
		self.joy = Joy()
		self.state = ArmState()
		self.joints = JointState()
		self.science = Science()
		self.grip = 0
		
		# Initialize state; default = JointControl & Medium
		self.state.mode = 'Science' # 'JointControl', 'IK Arm - Base,Tool', 'IK Arm - Tool,Tool'
		self.state.speed = 'Slow' # Slow, Med, Fast
		self.state.kill = False

		# Init Science Stuff
		self.science.elevator = 0
		self.science.plunge = 0
		self.science.drill = 0
		self.science.plate = 0

	# Publishers and Subscribers

		# Subscribe to /joy_arm /pose_cmd
		self.sub_joy = rospy.Subscriber('/joy_arm', Joy, self.joyCallback)
	
		# Publish
		self.pub_science = rospy.Publisher('/science_cmd', Science, queue_size = 10)
		self.pub_state = rospy.Publisher('/arm_state_cmd', ArmState, queue_size = 10)


		# self.pub_grip = rospy.Publisher('/grip', Int8, queue_size = 10)
	   
	##### Callbacks ###########

	def joyCallback(self,msg):
		self.joy = msg
		if self.joy.buttons[9] == 1:
			pass
			# if self.check==False:            
			#     self.check=True
			# else:
			#     self.check=False

	# Functions
	def check_method(self):
		# Check to see whether driving or using arm and return case
		# [A, B, X, Y] = buttons[0, 1, 2, 3]
		y = self.joy.buttons[3] # toggle between modes
		home = self.joy.buttons[8]
		
		# if y == 1: # UNCOMMENT THIS TO SWITCH BETWEEN MODES
		#     if self.state.mode == 'JointControl':
		#         self.state.mode = 'IK Arm - Base,Tool'
		#     elif self.state.mode == 'IK Arm - Base,Tool':
		#         self.state.mode = 'IK Arm - Tool,Tool'
		#     else:
		#         self.state.mode = 'JointControl'
			# time.sleep(.25)
			# rospy.loginfo(self.state.mode)
			
		# Implement Kill Switch
		if home == 1:
			if self.state.kill == False:
				self.state.kill  = True
			else:
				self.state.kill  = False
			time.sleep(.25)
		
		# Publish state commands
		self.pub_state.publish(self.state)

	def speed_check(self):
		# toggle between arm speeds
		rb = self.joy.buttons[5]
		if rb == 1:
			if self.state.speed == 'Slow':
				self.state.speed = 'Med'
			elif self.state.speed == 'Med':
				self.state.speed = 'Fast'
			elif self.state.speed == 'Fast':
				self.state.speed = 'Slow'
			time.sleep(.25)

	def gripper(self):
		rt = (1 - self.joy.axes[5])/2.0
		lt = (1 - self.joy.axes[2])/2.0
		
		threshold = 0.1
		
		if rt >= threshold: # open
			self.grip = rt*100
		elif lt >= threshold: # close
			self.grip = -lt*100
		else:
			self.grip = 0
		

	# ==========================================================================
	# Xbox Science Control ===============================================
	# ==========================================================================
	def science_cmd(self):
		
		# Speed Check
		self.speed_check()

		# Set corresponding rate
		if self.state.speed == 'Fast':
			MAX_RATE = 100
		elif self.state.speed == 'Med':
			MAX_RATE = 50
		elif self.state.speed == 'Slow':
			MAX_RATE = 20

		# Calculate how to command arm (position control)
		DEADZONE = 0.1
		
		# Set axes
		left_joy_up = self.joy.axes[1]
		left_joy_right = self.joy.axes[0]
		right_joy_up = self.joy.axes[4]
		right_joy_right = self.joy.axes[3]
		hat_up = self.joy.axes[7]
		hat_right = self.joy.axes[6]
		lb = self.joy.buttons[4]
		x_button = self.joy.buttons[2]

		
		# # Set axis to zero in deadzone
		# for i in range(0,len(axes)):
		# 	if abs(axes[i])<DEADZONE:
		# 		axes[i] = 0
		
		self.science.elevator = self.science.elevator + MAX_RATE*left_joy_up
		if self.science.elevator < 0:
			self.science.elevator = 0
		elif self.science.elevator > 4095:
			self.science.elevator = 4095
		self.science.plunge = self.science.plunge + MAX_RATE*right_joy_up
		if self.science.plunge < 0:
			self.science.plunge = 0
		elif self.science.plunge > 4095:
			self.science.plunge = 4095

		# DRILL
		if x_button and (self.science.drill == 0):
			self.science.drill = 1
			rospy.sleep(0.5)
		elif x_button and self.science.drill:
			self.science.drill = 0
			rospy.sleep(0.5)

		# PLATE
		if lb:
			self.science.plate = 0
		elif (hat_up==1):
			self.science.plate = 1000
		elif (hat_up == -1):
			self.science.plate = 3000
		elif (hat_right == 1):
			self.science.plate = 4000
		elif (hat_right == -1):
			self.science.plate = 2000

		# Publish arm commands
		self.pub_science.publish(self.science)

	# ==========================================================================
	# Main ===============================================
	# ==========================================================================
if __name__ == '__main__':
	# init ROS node
	rospy.init_node('xbox_science_control')
	
	# set rate
	hz = 10.0#60.0
	rate = rospy.Rate(hz)

	# init Arm_xbox object
	xbox = Arm_XBOX()
	
	# Loop
	while not rospy.is_shutdown():

		# Start when Xbox controller recognized
		if len(xbox.joy.buttons) > 0:
			
			# every time check toggle of state
			xbox.check_method()

			# check for kill switch (True = Killed)
			if xbox.state.kill  == False:

				# call appropriate function for state
				# defaults to JointControl
				if xbox.state.mode == 'Science':
					xbox.science_cmd()
				else:
					rospy.logwarn('ERROR: SHOULD BE IN SCIENCE MODE????')
		#else:
		 #   xbox.pub_joints.publish(xbox.joints)

		rate.sleep()


