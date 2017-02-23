
import sys
import numpy as np
import param as P
import math
import rospy

'''
PID Controller to hone in on gate. Controls x1-x2, where x1 is the distance of
the centroid of the ball from the left edge of the image and x2 is the distance
of the centroid of the ball from the left edge of the image

param.py contains gains and other parameters
'''
class controllerPID:

	def __init__(self):
		# Instantiates the PID_ctrl object
		# Control of x1-x2
		self.xCtrl = xPID_ctrl(P.x_kp,P.x_kd,P.x_ki,P.x0,P.limit)

		# self.my_pub = rospy.Publisher('/my_states', MyStates, queue_size=10 )
		# self.my_state = MyStates()
		# kp is the proportional gain
		# kd is the derivative gain
		# ki is the integral gain
		# y0 is the initial position of the state

	def getInputs(self,y_r,y):
		# y_r is the referenced input
		# y is the current state
		# longitudinal
		x_r = y_r[0]

		# states
		x = y[0]


		# longitudinal

		twist = self.xCtrl.xPID_loop(x_r,x) # Calculate the force output

		#I'm pretty sure (since the current plan is to stay in place) that
		#once F is returned, we just set R = F, L = -F (we can apply a scaling)
		#because if x1<x2, state is negative and we want to increase it

		return twist


class xPID_ctrl:
	def __init__(self,kp,kd,ki,x0,limit):
		self.differentiator = 0.0    # Difference term
		self.integrator = 0.0
		self.x_d1 = x0       # Delayed y output
		self.error_d1 = 0.0          # Delayed error
		self.kp = kp                 # Proportional control gain
		self.kd = kd 				# Derivative control gain
		self.ki = ki                # Integral control gain
		self.limit = limit           # Maxiumum force


	def xPID_loop(self,x_r,x):
		# Compute the current error
		error = x_r - x
		# rospy.logwarn(str(error))

		# UPIDate Differentiator
		a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
		a2 = 2/(2*P.sigma+P.Ts)
		self.differentiator = a1*self.differentiator \
		                  + a2*(x -self.x_d1)
		# Update Integrator
		if abs(self.differentiator) <0.05:
			self.integrator = self.integrator + (P.Ts/2.0)*(error+self.error_d1)

		# UPIDate error_d1
		self.error_d1 = error
		self.x_d1 = x

		u_unsat = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator

		u_sat = self.saturate(F_unsat)

		# anti windup
		if self.ki != 0:
			self.integrator += P.Ts/self.ki*(F_sat-F_unsat)

		# return F_sat
		return u_sat

	def saturate(self,u):
		if abs(u) > self.limit:
			u = self.limit*np.sign(u)
			print 'Force Saturated'
			rospy.logwarn('Force Saturated')
		return u
