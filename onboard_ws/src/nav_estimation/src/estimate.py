#!/usr/bin/env python

import rospy
from math import *
#from ctypes import c_ushort
from rover_msgs.msg import RoverState, Drive, NavState
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
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
		self.estimate.base_latitude = 0
		self.estimate.base_longitude = 0
		self.estimate.base_altitude = 0

		self.phat = 0
		self.qhat = 0
		self.rhat = 0

		# GPS Init
		self.gps_init = False

		# Init Kalman Filter stuff
		self.N = 10 # Steps for Kalman Filter
		self.xhat = np.zeros(3) # pn, pe, psi

		self.L = np.zeros(3)
		
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

		self.time_old = rospy.get_rostime()
		self.time_new = rospy.get_rostime()

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
		self.drive_gain = 1.7/100 # measured 1.7 meters/s when commanded full speed
		self.turn_gain = 0.004

		self.lpf_a = 0.0

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
		# print 'GPS Callback'
		
		# init GPS
		if not self.gps_init:
			# print 'GPS INIT'
			self.gps_init = True
			self.estimate.base_latitude = msg.latitude
			self.estimate.base_longitude = msg.longitude
			self.estimate.base_altitude = msg.altitude
			self.time_old = rospy.get_rostime()
		else:
			# print 'GPS Input'
			self.gps_n = self.EARTH_RADIUS*(msg.latitude-self.estimate.base_latitude)*np.pi/180.0
			self.gps_e = self.EARTH_RADIUS*cos(self.estimate.base_latitude*np.pi/180.0)*(msg.longitude-self.estimate.base_longitude)*np.pi/180.0
			self.gps_h = msg.altitude - self.estimate.base_altitude

			self.gps_new = True
			self.update()
	
	def imuCallback(self, msg):
		
		# Get time
		self.time_new = rospy.get_rostime()
		Ts = (self.time_new.secs - self.time_old.secs) + (self.time_new.nsecs - self.time_old.nsecs)/1e9
		self.time_old = self.time_new

		if (Ts > 0.5):
			rospy.logwarn('Estimation Time Step was Too Big: %f', Ts)
		else:

			# Compute alpha for time step
			alpha = exp(-self.lpf_a*Ts)

			self.phat = alpha*self.phat + (1-alpha)*msg.angular_velocity.x
			self.qhat = alpha*self.qhat + (1-alpha)*msg.angular_velocity.y
			self.rhat = alpha*self.rhat + (1-alpha)*msg.angular_velocity.z

			q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
			euler = tf.transformations.euler_from_quaternion(q)
			self.estimate.phi = euler[0];
			self.estimate.theta = euler[1];
			self.estimate.psi = -euler[2];

			self.prediction(Ts)

			if self.gps_init:
				self.publish()

	def driveCallback(self, msg):
		self.Vwhat = (msg.lw + msg.rw)*self.drive_gain/2.0
		self.omegahat = (msg.lw - msg.rw)*self.turn_gain

	def prediction(self, Ts):
		# print 'Prediction'
		
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
		# print 'Update'

		# Init identity
		I = np.identity(3)

		if self.gps_new:

			# Reset New measurement flag
			self.gps_new = False

			# GPS North
			h = self.xhat[0] #pn
			C = np.zeros(3)
			C[0] = 1
			denom = (self.R[0][0] + np.dot(np.dot(C.transpose(),self.P),C))
			self.L = (np.dot(self.P,C))/denom
			self.P = (I - np.dot(np.dot(self.L,C.transpose()),self.P))
			self.xhat = self.xhat + self.L*(self.gps_n - h)
			# print 'GPS N L:'
			# print self.L

			# # GPS East
			h = self.xhat[1] #pe
			C = np.zeros(3)
			C[1] = 1
			denom = (self.R[1][1] + np.dot(np.dot(C.transpose(),self.P),C))
			self.L = (np.dot(self.P,C))/denom
			self.P = (I - np.dot(np.dot(self.L,C.transpose()),self.P))
			self.xhat = self.xhat + self.L*(self.gps_e - h)
			# print 'GPS E L:'
			# print self.L

		# Error if estimates aren't realistic
		if (self.xhat[0] > 2000 or self.xhat[0] < -2000 or self.xhat[1] < -2000 or self.xhat[1] > 2000):
			rospy.logwarn('GPS Estimates Too Big Pn: %f, Pe: %f', self.xhat[0], self.xhat[1])

	def publish(self):
		self.estimate.position[0] = self.xhat[0]
		self.estimate.position[1] = self.xhat[1]
		self.estimate.position[2] = self.gps_h # not estimated
		self.estimate.Vw = self.Vwhat # input
		self.estimate.Vg = self.omegahat # omega is published as Vg for now
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

