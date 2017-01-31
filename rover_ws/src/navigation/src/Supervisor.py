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
        rate = rospy.Rate(hz)

        # Subscribe to /joy_arm /pose_cmd
        self.sub_estimate = rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
        self.sub_navdata = rospy.Subscriber('/navigation_data', NavigationData, self.navdataCallback)
        self.sub_obst = rospy.Subscriber('/obstacles', Obstacles, self.obstacleCallback)

        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # Instantiate Classes
        self.robot = RobotState()
        self.gtg = GoToGoal()


    def odomCallback(self,msg):
        a = 1

    def navdataCallback(self,msg):
        a = 1

    def obstacleCallback(self,msg):
        a = 1





if __name__ == '__main__':
    Sup = Supervisor()


