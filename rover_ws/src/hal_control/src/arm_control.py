#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
from rover_msgs.msg import ArmState
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
import time
import numpy as np
from urdf_parser_py.urdf import URDF
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
import random


class Arm_XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
        self.state = ArmState()
        self.joints = JointState()
        #self.joints_cart = Float32MultiArray()
        self.pose_current = Pose()
        self.pose_cmd = Pose()
        self.grip = 0
        
        # Initialize state; default = JointControl & Medium
        self.state.mode = 'JointControl' # JointControl, VelControl IK Arm
        self.state.speed = 'Med' # Slow, Med, Fast
        self.state.kill = False

        # Initialize joints = instance of JointState
        self.joints.header = Header()
        self.joints.header.stamp = rospy.Time.now()
        self.joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints.effort = []

        # Initialize FK
        self.init_ik = True
        self.pose_current.position.x = 0.0
        self.pose_current.position.y = 0.0
        self.pose_current.position.z = 0.0

    # Publishers and Subscribers

    	# Subscribe to /joy_arm /pose_cmd
        self.sub_joy = rospy.Subscriber('/joy_arm', Joy, self.joyCallback)
        self.sub_pose_cmd= rospy.Subscriber('/pose_cmd', Pose, self.ikPoseCallback)
        self.sub_joint_cmd_ik = rospy.Subscriber('/joint_cmd_ik',JointState, self.ikjointCallback)
	
        # Publish /arm_state_cmd; /joint_cmd; /grip; /joint_cart_cmd
        self.pub_state = rospy.Publisher('/arm_state_cmd', ArmState, queue_size = 10)
        self.pub_joints = rospy.Publisher('/joint_cmd', JointState, queue_size = 10)
        self.pub_joint_ik = rospy.Publisher('/joint_ik', JointState, queue_size = 10)
        self.pub_pose_ik = rospy.Publisher('/pose_ik', Pose, queue_size = 10)
        self.pub_grip = rospy.Publisher('/grip', Int8, queue_size = 10)
       
    ##### Callbacks ###########

    def joyCallback(self,msg):
        self.joy=msg
        if self.joy.buttons[9] == 1:
            if self.check==False:            
                self.check=True
            else:
                self.check=False

    def ikPoseCallback(self,msg):
        self.pose_current = msg

    def ikjointCallback(self, msg):
        self.joints.position = msg.position
        self.pub_joints.publish(self.joints)

    # Functions
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes
        home = self.joy.buttons[8]
        
        if y == 1:
            if self.state.mode == 'JointControl':
                self.state.mode = 'VelControl'
            elif self.state.mode == 'VelControl':
                self.state.mode = 'IK Arm'
            else:
                self.state.mode = 'JointControl'
            time.sleep(.25)
            rospy.loginfo(self.state.mode)
            
        # Implement Kill Switch
        if home == 1:
            if self.state.kill == False:
                self.state.kill  = True
            else:
                self.state.kill  = False
            time.sleep(.25)
        
        # Publish state commands
        self.pub_state.publish(self.state)

    def speed_check(self):
    	# toggle between arm speeds
        rb = self.joy.buttons[5]
        if rb == 1:
            if self.state.speed == 'Slow':
                self.state.speed = 'Med'
            elif self.state.speed == 'Med':
                self.state.speed = 'Fast'
            elif self.state.speed == 'Fast':
                self.state.speed = 'Slow'
            time.sleep(.25)

    def gripper(self):
        rt = (1 - self.joy.axes[5])/2.0
        lt = (1 - self.joy.axes[2])/2.0
        
        threshold = 0.1
        
        if rt >= threshold: # open
            self.grip = rt*100
        elif lt >= threshold: # close
            self.grip = -lt*100
        else:
            self.grip = 0

    # ==========================================================================
    # INVERSE KINEMATICS CONTROL ===============================================
    # ==========================================================================
    def arm_IK(self):

    	# read in & initialize position of arm
    	# if first time
        if self.init_ik:
            # Publish current joint position
            self.pub_joint_ik.publish(self.joints)
            time.sleep(.25)
            self.pose_cmd = self.pose_current
            self.init_ik = False
    	# FK on last commanded angles
        
    	###### change pose with Xbox
        # Speed Check
        self.speed_check()
        
        # Set corresponding rate
        if self.state.speed == 'Fast':
            MAX_RATE = .01
        elif self.state.speed == 'Med':
            MAX_RATE = .001
        elif self.state.speed == 'Slow':
            MAX_RATE = .0001

        # Calculate how to command arm (position control)
        DEADZONE = 0.1
        
        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]
        
        # make array of axes
        axes = [left_joy_right, left_joy_up, hat_up,
            hat_right, right_joy_up, right_joy_right]
        
        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0
                
        # update pose_cmd with result from IK Node
        self.pose_cmd = self.pose_current
        
        # Update Cartesian Positions
        self.pose_cmd.position.x -= axes[0]*MAX_RATE
        self.pose_cmd.position.y += axes[1]*MAX_RATE
        self.pose_cmd.position.z += axes[4]*MAX_RATE
        self.pose_cmd.orientation.x = 0
        self.pose_cmd.orientation.y = 0
        self.pose_cmd.orientation.z = 0
        self.pose_cmd.orientation.w = 1
        
        print self.pose_cmd.position.x, -axes[0]*MAX_RATE, left_joy_right
        
    	# send pose to IK
        self.pub_pose_ik.publish(self.pose_cmd)
        
        #print self.pose_cmd
        

    # ==========================================================================
    # Xbox Arm Control ===============================================
    # ==========================================================================
    def joint_cmd(self):
        
        # Speed Check
        self.speed_check()

        # Set corresponding rate
        if self.state.speed == 'Fast':
        	MAX_RATE = 10*np.pi/180
        elif self.state.speed == 'Med':
        	MAX_RATE = 5*np.pi/180
        elif self.state.speed == 'Slow':
        	MAX_RATE = 2*np.pi/180

        # Calculate how to command arm (position control)
        DEADZONE = 0.1
        
        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]
        
        # make array of axes
        axes = [left_joy_right, left_joy_up, hat_up,
            hat_right, right_joy_up, right_joy_right]
        
        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0
                
        # Update joint angles
        for i in range(0,6):
            self.joints.position[i] += axes[i]*MAX_RATE
        
        # Set joint angle limits
        for i in range(0,len(self.joints.position)):
            if self.joints.position[i] > np.pi:
                self.joints.position[i] = np.pi
            elif self.joints.position[i] < -np.pi:
                self.joints.position[i] = -np.pi

        self.joints.header.stamp = rospy.Time.now()
        self.joints.header.frame_id = 'JointControl'                

        # Gripper
        self.gripper()

        # # Shovel
        # if self.joy.axes[2] < 0:
        #     self.cmd.shovel = self.cmd.shovel-10.0
        #     if self.cmd.shovel < 1000:
        #         self.cmd.shovel = 1000
        # elif self.joy.axes[5] < 0:
        #     self.cmd.shovel = self.cmd.shovel+10.0
        #     if self.cmd.shovel > 2000:
        #         self.cmd.shovel = 2000
        
        # set flag so IK knows must init when entered again
        self.init_ik = True

        # Publish arm commands
        self.pub_joints.publish(self.joints)
        self.pub_joint_ik.publish(self.joints)


    # ==========================================================================
    # Velocity Arm Control ===============================================
    # ==========================================================================
    def vel_cmd(self):
        
        # Speed Check
        self.speed_check()

        # Set corresponding rate
        if self.state.speed == 'Fast':
        	MAX_RATE = 100
        elif self.state.speed == 'Med':
        	MAX_RATE = 75
        elif self.state.speed == 'Slow':
        	MAX_RATE = 50

        # Calculate how to command arm (position control)
        DEADZONE = 0.1
        
        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]
        
        # make array of axes
        axes = [left_joy_right, left_joy_up, hat_up,
            hat_right, right_joy_up, right_joy_right]
        
        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0
                
        # Update joint angles
        for i in range(0,6):
            self.joints.velocity[i] = axes[i]*MAX_RATE
        
#        # Set joint angle limits
#        for i in range(0,len(self.joints.position)):
#            if self.joints.position[i] > np.pi:
#                self.joints.position[i] = np.pi
#            elif self.joints.position[i] < -np.pi:
#                self.joints.position[i] = -np.pi

        self.joints.header.stamp = rospy.Time.now()
        self.joints.header.frame_id = 'VelControl'        

        # Publish arm commands
        self.pub_joints.publish(self.joints)

    # ==========================================================================
    # Main ===============================================
    # ==========================================================================
if __name__ == '__main__':
    # init ROS node
    rospy.init_node('xbox_arm_control')
    
    # set rate
    hz = 60.0
    rate = rospy.Rate(hz)

    # init Arm_xbox object
    xbox = Arm_XBOX()
    
    # Loop
    while not rospy.is_shutdown():

    	# Start when Xbox controller recognized
        if len(xbox.joy.buttons) > 0:
            
            # every time check toggle of state
            xbox.check_method()

            # check for kill switch (True = Killed)
            if xbox.state.kill  == False:

	            # call appropriate function for state
	            # defaults to JointControl
	            if xbox.state.mode == 'JointControl':
	                xbox.joint_cmd()
	            elif xbox.state.mode == 'VelControl':
	                xbox.vel_cmd()
	            elif xbox.state.mode == 'IK Arm':
	                xbox.arm_IK()
	            else:
	                xbox.joint_cmd()
        #else:
         #   xbox.pub_joints.publish(xbox.joints)

        rate.sleep()

