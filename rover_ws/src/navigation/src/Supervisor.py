#!/usr/bin/env python

import numpy as np
import rospy
import time
import math
import tf
import sys
import tf.transformations as tr
from std_srvs.srv import Empty as srv_Empty
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rover_msgs.msg import ArmState
from rover_msgs.msg import NavigationData, Obstacles
from RobotState import RobotState
from gotogoal import GoToGoal, Stop


class Supervisor:
    def __init__(self, baseframe=False):
        # init ROS node
        rospy.init_node('navigation')

        # TF Listener
        self.listener = tf.TransformListener()

        # set rate
        hz = 10.0 # 60.0
        self.rate = rospy.Rate(hz)

        # Set Params
        dt = 0.01
        ki, kd, kp = 1, 10, 3
        sigma = 0.05
        params = Params(ki, kd, kp, dt, sigma)
        goal_distance = 0.1
        default_vel = 1

        # Instantiate Classes
        self.robot = RobotState(goal_distance, default_vel)
        self.gtg = GoToGoal(params)
        self.stop = Stop(params)

        # Other admin stuff
        self.msgread = {'odom': False, 'navdata': False}
        self.goal_in_base_frame = baseframe
        if baseframe:
            rospy.loginfo("Goal in Base Frame")
        else:
            rospy.loginfo('Goal in Odom Frame')

        # Subscribe to /joy_arm /pose_cmd
        self.sub_estimate = rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
        self.sub_navdata = rospy.Subscriber('/navigation_data', NavigationData, self.navdataCallback)
        self.sub_obst = rospy.Subscriber('/obstacles', Obstacles, self.obstacleCallback)

        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # Get initial pose
        pose = -1
        fails = 0
        while pose == -1:
            pose = self.base_transform()
            if pose == -1:
                fails += -pose
            if fails > 10000:
                rospy.logwarn('Transfrom from /odom to /base_link not found')
                pose = 0
        self.initial_pose = pose

        # Set Goal
        goal = [5, 5]
        self.robot.set_goal(goal)

        # Set up waypoint stuff
        self.cur_waypoint = 0
        self.wp_x = [goal[0]]
        self.wp_y = [goal[1]]

        # Set default controller
        self.control = self.gtg

        # Set up Services
        self.enable = False
        self.srv_start = rospy.Service('StartAuto', srv_Empty, self.start_auto)
        self.srv_stop = rospy.Service('StopAuto', srv_Empty, self.stop_auto)
        self.srv_reset = rospy.Service('ResetAuto', srv_Empty, self.reset_auto)


    def execute(self):
        while not rospy.is_shutdown():
            if self.ready() and self.enable:
                self.robot.set_goal(self.get_goal())
                self.check_states()
                v, w = self.control.get_outputs(self.robot)
                self.send_command(v, w)
                # print self.robot.dist_to_goal()
                # print self.get_goal()
                # print self.control.vector_to_goal
                # print self.control.get_angle()
                # print self.base_transform()
                self.rate.sleep()
            else:
                self.rate.sleep()
            

    def check_states(self):
        if self.control.name == "go to goal":
            if self.robot.at_goal():
                if self.cur_waypoint < len(self.wp_x)-1:
                    self.cur_waypoint += 1
                    self.control.pid.reset()
                    rospy.loginfo("New Goal: (%s)" % ', '.join(map(str, self.get_goal())))
                else:
                    self.control = self.stop
                    rospy.loginfo("AT GOAL!!!!")
                    self.cur_waypoint = 0

    def send_command(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_drive.publish(cmd)

    def base_transform(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return -1

    def odomCallback(self, msg):
        self.robot.pose = msg.pose.pose
        self.robot.twist = msg.twist.twist
        self.msgread['odom'] = True

    def navdataCallback(self, msg):
        self.wp_x = msg.wp_x
        self.wp_y = msg.wp_y
        self.msgread['navdata'] = True

    def obstacleCallback(self, msg):
        a = 1

    def ready(self):
        ready = True
        for key in self.msgread:
            ready = ready and self.msgread[key]
        return ready

    def get_goal(self):
        if self.goal_in_base_frame:
            goal_b = [self.wp_x[self.cur_waypoint], self.wp_y[self.cur_waypoint], 0, 1]
            (trans, rot) = self.initial_pose
            H = self.tr2matrix(trans, rot)
            goal_odom = np.dot(H, goal_b)
            goal = goal_odom[0:2].tolist()
        else:
            goal = [self.wp_x[self.cur_waypoint], self.wp_y[self.cur_waypoint]]
        return goal

    def tr2matrix(self,trans,rot):
        # Returns the homogenous transformation matrix of the /base_link in the /odom frame
        Htrans = tr.translation_matrix(trans)
        Hrot = tr.quaternion_matrix(rot)
        H = tr.concatenate_matrices(Htrans, Hrot)
        return H

    ############# Services ###################
    def start_auto(self, srv):
        self.enable = True
        rospy.loginfo('Starting Autonomous Mode')
    def stop_auto(self, srv):
        self.enable = False
        rospy.loginfo('Stopping Autonomous Mode')
    def reset_auto(self, srv):
        self.enable = False
        self.control = self.gtg
        self.cur_waypoint = 0
        rospy.loginfo('Resetting Autonomous Mode')


class Params:
    def __init__(self, ki=1, kd=0.1, kp=5, dt=0.01, sigma=0.05):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.sigma = sigma


if __name__ == '__main__':
    if len(sys.argv) == 2:
        baseframe = sys.argv[1]
    else:
        baseframe = False
    Sup = Supervisor(baseframe)
    Sup.execute()


