import numpy
import rospy
from math import *
from geometry_msgs.msg import Pose, Twist
from Controller import Controller

import tf.transformations as tr


class Backup(Controller):
    def __init__(self, params, backup_time=3, backup_twist=[-2,1]):
        Controller.__init__(self, params)
        self.vector_to_goal = [0, 0]
        self.name = "backup"

        self.start_time = rospy.get_time()
        self.run_time = backup_time
        self.backup_twist = backup_twist

    def start(self):
        self.start_time = rospy.get_time()

    def is_done(self):
        return rospy.get_time() - self.start_time > self.run_time


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
        if self.get_heading(robotstate) > 0: # Turning right
            w = self.backup_twist[1]*-1
        else:
            w = self.backup_twist[1]
        v = self.backup_twist[0]    

        return v, w


class Stop(Controller):
    def __init__(self, params):
        Controller.__init__(self, params)
        self.name = "stop"

    def get_outputs(self, robotstate):
        return 0, 0
