#!/usr/bin/env python

import rospy
from math import *
#from ctypes import c_ushort
from rover_msgs.msg import NavState
# from sensor_msgs.msg import NavSatFix, Imu
# from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from inertial_sense.msg import GPS
import time
import numpy as np
import tf


class Estimator():
	def __init__(self):
	# Variables
		self.estimate = NavState()
		
		# Initialize NavState
		self.estimate.position[0] = 0 # gps_n
		self.estimate.position[1] = 0 # gps_e
		self.estimate.position[2] = 0 # gps_h
		self.estimate.Vw = 0
		self.estimate.phi = 0
		self.estimate.theta = 0
		self.estimate.psi = 0
		self.estimate.chi = 0
		self.estimate.p = 0
		self.estimate.q = 0
		self.estimate.r = 0
		self.estimate.Vg = 0
		self.estimate.base_latitude = 0 # Hard code these in from the inertial sense
		self.estimate.base_longitude = 0 # Hard code these in from the inertial sense
		self.estimate.base_altitude = 0 # Hard code these in from the inertial sense

	
	# Publishers and Subscribers
		self.sub_ins = rospy.Subscriber('/ins', Odometry, self.insCallback)
		self.sub_gps = rospy.Subscriber('/imu', GPS, self.GPSCallback)
		# self.sub_drive = rospy.Subscriber('/drive_cmd', Drive, self.driveCallback)

		self.pub_state = rospy.Publisher('/estimate', NavState, queue_size = 10)

	# Functions
	def GPSCallback(self, msg):
		# print 'GPS Callback'
		pass
		# # init GPS
		# if not self.gps_init:
		# 	# print 'GPS INIT'
		# 	self.gps_init = True
		# 	self.estimate.base_latitude = msg.latitude
		# 	self.estimate.base_longitude = msg.longitude
		# 	self.estimate.base_altitude = msg.altitude
		# 	self.time_old = rospy.get_rostime()
		# else:
		# 	# print 'GPS Input'
		# 	self.gps_n = self.EARTH_RADIUS*(msg.latitude-self.estimate.base_latitude)*np.pi/180.0
		# 	self.gps_e = self.EARTH_RADIUS*cos(self.estimate.base_latitude*np.pi/180.0)*(msg.longitude-self.estimate.base_longitude)*np.pi/180.0
		# 	self.gps_h = msg.altitude - self.estimate.base_altitude

		# 	self.gps_new = True
		# 	self.update()
	
	def insCallback(self, msg):
		
		EARTH_RADIUS = 6378145.0

		self.estimate.position[0] = (msg.pose.pose.position.x*180.0)/(np.pi*EARTH_RADIUS) # lat
		self.estimate.position[1] = (msg.pose.pose.position.y*180.0)/(EARTH_RADIUS*np.pi*cos(0.0*np.pi/180.0)) # lon
		self.estimate.position[2] = msg.pose.pose.position.z # alt

		q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.estimate.phi = euler[0];
		self.estimate.theta = -euler[1];
		self.estimate.psi = -euler[2];
		self.estimate.chi = self.estimate.psi * 180.0/np.pi

	def publish(self):
		self.pub_state.publish(self.estimate)


	# ==========================================================================
	# Main ===============================================
	# ==========================================================================
if __name__ == '__main__':
	# init ROS node
	rospy.init_node('estimator')
	

	# set rate
	hz = 10.0
	rate = rospy.Rate(hz)
	
	# init estimate object
	estimate = Estimator()
	
	# Loop
	while not rospy.is_shutdown():

		estimate.publish()
		rate.sleep()

