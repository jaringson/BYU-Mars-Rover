import numpy as np
import rospy
import time
import math
import tf
import tf.transformations as tr
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rover_msgs.msg import ArmState
from rover_msgs.msg import NavigationData, Obstacles
from RobotState import RobotState
from gotogoal import GoToGoal, Stop


class Supervisor:
    def __init__(self):
        # init ROS node
        rospy.init_node('navigation')

        # set rate
        hz = 10.0 # 60.0
        self.rate = rospy.Rate(hz)

        # Subscribe to /joy_arm /pose_cmd
        self.sub_estimate = rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
        self.sub_navdata = rospy.Subscriber('/navigation_data', NavigationData, self.navdataCallback)
        self.sub_obst = rospy.Subscriber('/obstacles', Obstacles, self.obstacleCallback)

        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set Params
        dt = 0.01
        ki, kd, kp = 1, 0.1, 4
        sigma = 0.05
        params = Params(ki, kd, kp, dt, sigma)

        # Instantiate Classes
        self.robot = RobotState()
        self.gtg = GoToGoal(params)
        self.stop = Stop(params)

        # Set Goal
        goal = [5, 5]
        self.robot.set_goal(goal)

        # Set up waypoint stuff
        self.cur_waypoint = 0
        self.wp_x = [goal[0]]
        self.wp_y = [goal[1]]

        # Set default controller
        self.control = self.gtg

        # Other admin stuff
        self.msgread = {'odom': False, 'navdata': False}

    def execute(self):
        while not rospy.is_shutdown():
            if self.ready():
                self.robot.set_goal(self.get_goal())
                self.check_states()
                v, w = self.control.get_outputs(self.robot)
                self.send_command(v, w)
                # print self.robot.dist_to_goal()
                # print self.get_goal()
                # print self.control.vector_to_goal
                # print self.control.get_angle()
                self.rate.sleep()

    def check_states(self):
        if self.control.name == "go to goal":
            if self.robot.at_goal():
                if self.cur_waypoint < len(self.wp_x)-1:
                    self.cur_waypoint += 1
                    self.control.pid.reset()
                    print "New Goal: (%s)" % ', '.join(map(str, self.get_goal()))
                else:
                    self.control = self.stop
                    print "AT GOAL!!!!"

    def send_command(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_drive.publish(cmd)

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
        return [self.wp_x[self.cur_waypoint], self.wp_y[self.cur_waypoint]]



class Params:
    def __init__(self, ki=1, kd=0.1, kp=5, dt=0.01, sigma=0.05):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.sigma = sigma


if __name__ == '__main__':
    Sup = Supervisor()
    Sup.execute()


