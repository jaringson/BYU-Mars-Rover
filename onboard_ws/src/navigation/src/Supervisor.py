#!/usr/bin/env python

import numpy as np
import rospy
import time
import math
import tf
import sys
import tf.transformations as tr
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rover_msgs.msg import ArmState, NavState
from rover_msgs.msg import NavigationData, Obstacles
from rover_msgs.srv import WaypointSend, WaypointSendResponse
from rover_msgs.srv import PositionReturn, PositionReturnResponse
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
        ki, kd, kp = 1, 5, 1.5
        sigma = 0.05
        params = Params(ki, kd, kp, dt, sigma)
        goal_distance = 2
        default_vel = 1.5

        # Instantiate Classes
        self.robot = RobotState(goal_distance, default_vel)
        self.gtg = GoToGoal(params)
        self.stop = Stop(params)

        # Other admin stuff
        self.msgread = {'estimate': False, 'navdata': True}
        self.goal_in_base_frame = baseframe
        if baseframe:
            rospy.loginfo("Goal in Base Frame")
        else:
            rospy.loginfo('Goal in Odom Frame')

        # Subscribe to /joy_arm /pose_cmd
        self.sub_navstate = rospy.Subscriber('/estimate', NavState, self.estimateCallback)
        self.sub_estimate = rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
        self.sub_navdata = rospy.Subscriber('/navigation_data', NavigationData, self.navdataCallback)
        self.sub_obst = rospy.Subscriber('/obstacles', Obstacles, self.obstacleCallback)

        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # Get initial pose
        # pose = -1
        # fails = 0
        # while pose == -1:
        #     pose = self.base_transform()
        #     if pose == -1:
        #         fails += -pose
        #     if fails > 10000:
        #         rospy.logwarn('Transfrom from /odom to /base_link not found')
        #         pose = 0
        self.initial_pose = NavState()
        self.initial_pose.base_latitude = -9999
        self.initial_pose.base_longitude = -9999

        # Set Goal
        goal = [0, 0]
        self.robot.set_goal(goal)

        # Set up waypoint stuff
        self.cur_waypoint = 0
        self.wp_x = [goal[0]]
        self.wp_y = [goal[1]]

        # Set default controller
        self.control = self.gtg

        # Set up Services
        self.enable = False
        self.srv_new = rospy.Service('NewWaypoints', WaypointSend, self.new_waypoints)
        self.srv_start = rospy.Service('StartAuto', Empty, self.start_auto)
        self.srv_stop = rospy.Service('StopAuto', Empty, self.stop_auto)
        self.srv_reset = rospy.Service('ResetAuto', Empty, self.reset_auto)

    # Main Function
    # Continuous loop that gets the command from the controller and sends it out
    def execute(self):
        while not rospy.is_shutdown():

            print self.robot.dist_to_goal()

            if self.ready() and self.enable:
                # Update goal
                self.robot.set_goal(self.get_goal())

                # Check for State Transition
                self.check_states()

                # Get commands from controller
                v, w = self.control.get_outputs(self.robot)

                # Send commands to robot
                self.send_command(v, w)

                # Execute at specified rate
                self.rate.sleep()
            else:
                self.rate.sleep()
            
    # Checks for transitions between states
    def check_states(self):

        # Go to Goal State
        if self.control.name == "go to goal":
            
            # Found goal
            if self.robot.at_goal():
                # Load Next Waypoing
                if self.cur_waypoint < len(self.wp_x)-1:
                    self.cur_waypoint += 1
                    self.control.pid.reset()
                    rospy.loginfo("New Goal: (%s)" % ', '.join(map(str, self.get_goal())))
                # At Final Goal
                else:
                    self.control = self.stop
                    rospy.loginfo("AT GOAL!!!!")
                    self.cur_waypoint = 0

    # Send velocities to wheel_controller
    def send_command(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_drive.publish(cmd)

    # Get the transform to the base
    # Used to get the initial pose for points in base frame
    def base_transform(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return -1
      
    ############# CALLBACKS #############
    def estimateCallback(self, msg):
        self.robot.state = msg
        self.msgread['estimate'] = True

    def odomCallback(self, msg):
        self.robot.pose = msg.pose.pose
        self.robot.twist = msg.twist.twist
        self.msgread['estimate'] = True

    # Waypoints
    def navdataCallback(self, msg):
        self.wp_x = msg.wp_x
        self.wp_y = msg.wp_y
        self.msgread['navdata'] = True

    def obstacleCallback(self, msg):
        a = 1

    ############# USEFUL FUNCTIONS #############
    # Checks if all the inital messages have been read in
    def ready(self):
        ready = True
        for key in self.msgread:
            ready = ready and self.msgread[key]
        return ready

    # Returns the current goal
    def get_goal(self):
        if self.goal_in_base_frame:
            # Transform point in base frame to global frame
            goal_b = [self.wp_x[self.cur_waypoint], self.wp_y[self.cur_waypoint], 0, 1]
            (trans, rot) = self.initial_pose
            H = self.tr2matrix(trans, rot)
            goal_odom = np.dot(H, goal_b)
            goal = goal_odom[0:2].tolist()
        else:
            goal = [self.wp_x[self.cur_waypoint], self.wp_y[self.cur_waypoint]]
        return goal

    # Returns the homogenous transformation matrix of the /base_link in the /odom frame
    def tr2matrix(self,trans,rot):
        Htrans = tr.translation_matrix(trans)
        Hrot = tr.quaternion_matrix(rot)
        H = tr.concatenate_matrices(Htrans, Hrot)
        return H

    ############# Services ###################
    def start_auto(self, srv):
        self.enable = True
        rospy.loginfo('Starting Autonomous Mode')
        return EmptyResponse()
    def stop_auto(self, srv):
        self.enable = False
        rospy.loginfo('Stopping Autonomous Mode')
        return EmptyResponse()
    def reset_auto(self, srv):
        self.enable = False
        self.control = self.gtg
        self.cur_waypoint = 0
        rospy.loginfo('Resetting Autonomous Mode')
        return EmptyResponse()
    def new_waypoints(self, srv):
        self.control = self.gtg
        self.cur_waypoint = 0
        self.wp_x = srv.wp_x
        self.wp_y = srv.wp_y
        msg = 'New Waypoints Received'
        if not self.enable:
            self.enable = True
            msg += '. Starting Autonomous Mode'
        # print srv.source
        response = WaypointSendResponse(True)
        rospy.loginfo(msg)
        return response


class Params:
    def __init__(self, ki=1, kd=0.1, kp=5, dt=0.01, sigma=0.05):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.sigma = sigma


if __name__ == '__main__':
    # Pass in True to place goal in the base frame of the robot and not the global frame
    if len(sys.argv) == 2:
        baseframe = sys.argv[1]
    else:
        baseframe = False
    Sup = Supervisor(baseframe)
    Sup.execute()


