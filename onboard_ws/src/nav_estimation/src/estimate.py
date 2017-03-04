#!/usr/bin/env python

import rospy
from math import *
#from ctypes import c_ushort
from rover_msgs.msg import RoverState, Drive, NavState
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
import time
import numpy as np


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
		self.estimate.base_latitude = 0
		self.estimate.base_longitude = 0
		self.base_altitude = 0

		# GPS Init
		self.gps_init = False

		# Init Kalman Filter stuff
		self.N = 10 # Steps for Kalman Filter
		self.xhat = np.zeros(3) # pn, pe, psi
		
		self.P = np.identity(3)
		self.P[0][0] = 0.03 # pn
		self.P[1][1] = 0.03 # pe
		self.P[2][2] = 5.0*np.pi/180.0 # psi
		
		self.Q = np.identity(3)
		self.Q[0][0] = 0.0001 # pn
		self.Q[1][1] = 0.0001 # pe
		self.Q[2][2] = 0.0001 # psi
		
		sigma_n_gps = 0.21
		sigma_h_gps = 0.4
		self.R = np.identity(3)
		self.R[0][0] = sigma_n_gps**2 # pn
		self.R[1][1] = sigma_h_gps**2 # pe
		self.R[2][2] = 0.0001 # psi # FIXXX THISSSSSS

		self.time_old = 0
		self.time_new = 0

		# Callback transfers
		self.Vwhat = 0
		self.omegahat = 0
		self.gps_n = 0
		self.gps_e = 0
		self.gps_h = 0

		# New data flags
		self.gps_new = False
		self.imu_new = False

		# GAINS
		self.drive_gain = 0.003511
		self.turn_gain = 0.004
		self.alpha = 0.0

		# Constants
		self.EARTH_RADIUS = 6378145.0
		self.gravity = 9.81


	# Publishers and Subscribers
		self.sub_gps = rospy.Subscriber('/fix', NavSatFix, self.gpsCallback)
		self.sub_imu = rospy.Subscriber('/imu', Imu, self.imuCallback)
		self.sub_drive = rospy.Subscriber('/drive_cmd', Drive, self.driveCallback)

		self.pub_state = rospy.Publisher('/estimate', NavState, queue_size = 10)

	# Functions
	def gpsCallback(self, msg):
		
		# init GPS
		if not self.gps_init:
			self.gps_init = True
			self.estimate.base_latitude = msg.latitude
			self.estimate.base_longitude = msg.longitude
			self.estimate.base_altitude = msg.altitude
		else:
			self.gps_n = self.EARTH_RADIUS*(msg.latitude-self.estimate.base_latitude)*np.pi/180.0
			self.gps_e = self.EARTH_RADIUS*cos(self.base_latitude*np.pi/180.0)*(msg.longitude-self.base_longitude)*np.pi/180.0
			self.gps_h = msg.altitude - self.base_altitude
	
	def imuCallback(self, msg):
		pass

	def driveCallback(self, msg):
		self.Vwhat = (msg.lw + msg.rw)*self.drive_gain
		self.omegahat = (msg.lw - msg.rw)*self.turn_gain

	def prediction(self, Ts):
		
		for i in range(0, self.N):

			f = np.zeros(3) # xdot = f(x,u)
			f[0] = self.Vwhat*cos(self.estimate.psi) # pndot
			f[1] = self.Vwhat*sin(self.estimate.psi) # pedot
			f[2] = self.omegahat # psidot

			A = np.zeros([3,3]) # d/dx(f(x,u))
			A[0][2] = -self.Vwhat*sin(self.estimate.psi)
			A[1][2] = self.Vwhat*cos(self.estimate.psi)

			self.xhat += (Ts/self.N)*f
			self.P += (Ts/self.N)*(np.multiply(A,self.P) + np.multiply(self.P, A.transpose()) + self.Q)


	def update(self):
		
		# Init identity
		I = np.identity(3)

	def publish(self):
		self.prediction(0.05)
		self.pub_state.publish()


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

