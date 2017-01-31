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
from gotogoal import GoToGoal


class Supervisor:
    def __init__(self):
        # init ROS node
        rospy.init_node('navigation')

        # set rate
        hz = 10.0  # 60.0
        self.rate = rospy.Rate(hz)

        # Subscribe to /joy_arm /pose_cmd
        self.sub_estimate = rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
        self.sub_navdata = rospy.Subscriber('/navigation_data', NavigationData, self.navdataCallback)
        self.sub_obst = rospy.Subscriber('/obstacles', Obstacles, self.obstacleCallback)

        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Set Params
        dt = 0.01
        ki, kd, kp = 1, 0.1, 5
        sigma = 0.05
        params = Params(ki, kd, kp, dt, sigma)

        # Instantiate Classes
        self.robot = RobotState()
        self.gtg = GoToGoal(params)

        # Set Goal
        goal = [-3, -3]
        self.robot.set_goal(goal)

    def execute(self):
        while not rospy.is_shutdown():
            #print self.robot.get_heading()
            v, w = self.gtg.get_outputs(self.robot)
            #print v, w
            self.send_command(v, w)
            print self.robot.dist_to_goal()
            self.rate.sleep()

    def send_command(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_drive.publish(cmd)


    def odomCallback(self, msg):
        self.robot.pose = msg.pose.pose
        self.robot.twist = msg.twist.twist

    def navdataCallback(self, msg):
        a = 1

    def obstacleCallback(self, msg):
        a = 1



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


