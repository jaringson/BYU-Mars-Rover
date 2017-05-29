import numpy
from math import *
from geometry_msgs.msg import Pose, Twist
from Controller import Controller
from PID import PID

import tf.transformations as tr


class Downhill(Controller):
    def __init__(self, params, speed=0.75):
        Controller.__init__(self, params)
        self.vector_to_goal = [0, 0]
        self.name = "downhill"
        self.speed = speed

        self.v_pid = PID(params)

    def get_heading(self, robotstate):
        """Return the heading as the angle to goal in the robot's reference frame"""
        
        # Extract goal
        x_goal, y_goal = robotstate.goal[0], robotstate.goal[1]
        # print x_goal, y_goal

        # Robot pose
        x_cur, y_cur, theta = robotstate.get_pose()
        
        # Vector to Goal
        self.vector_to_goal = [x_goal-x_cur, y_goal-y_cur]
        angle = self.get_angle()
        return angle
        
    def get_angle(self):
        return atan2(self.vector_to_goal[1], self.vector_to_goal[0])

    def get_outputs(self, robotstate):
        inputs = [0,      # desired heading
                  robotstate.get_roll()*pi/180.0,          # current heading
                  0]                                 # time

        w_max = 1
        w = self.pid.execute(inputs)*-1
        if abs(w) > w_max:
            w = w_max*numpy.sign(w)
          

        inputs_v = [self.speed, # Desired downhill speed
                    numpy.sign(robotstate.Vg)*robotstate.state.Vg, # Current speed
                    0]
        v = self.v_pid.execute(inputs_v)
        print v

        return v, w

