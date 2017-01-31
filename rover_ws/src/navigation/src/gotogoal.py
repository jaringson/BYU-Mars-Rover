import numpy
from math import *
from geometry_msgs.msg import Pose, Twist
from Controller import Controller

import tf.transformations as tr

class GoToGoal(Controller):
    def __init__(self, params):
        Controller.__init__(self, params)
        self.vector_to_goal = [0,0]

    def get_heading(self, robotstate):
        """Return the heading as the angle to goal in the robot's reference frame"""
        
        # Extract goal
        x_goal, y_goal = robotstate.goal[0], robotstate.goal[1]
        
        # Robot pose
        x_cur, y_cur, theta = robotstate.get_pose()
        
        # Vector to Goal
        self.vector_to_goal = [x_goal-x_cur, y_goal-y_cur]
        angle = self.get_angle()
        return angle
        
    def get_angle(self):
        return atan2(self.vector_to_goal[1], self.vector_to_goal[0])

    def get_outputs(self, robotstate):
        inputs = [self.get_heading(robotstate),      # desired heading
                  robotstate.get_heading(),          # current heading
                  0]                                 # time

        w = self.pid.execute(inputs)
        v = 0.5

        return v, w
        
