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
import tf
import tf.transformations as tr

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
        self.state.mode = 'JointControl' # 'JointControl', 'IK Arm - Base,Tool', 'IK Arm - Tool,Tool'
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
        # self.robot = URDF.from_parameter_server()
        # #self.robot = URDF.from_xml_file('/home/halrover/BYU-Mars-Rover/rover_ws/src/hal_description/urdf/hal.urdf')
        # self.tree = kdl_tree_from_urdf_model(self.robot)
        # self.base_link = self.robot.get_root()
        # self.end_link = self.robot.link_map.keys()[0]
        # self.chain = self.tree.getChain(self.base_link, self.end_link)
        # self.kdl_kin = KDLKinematics(self.robot, self.base_link, self.end_link)
        # self.pose = []

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
       
    # Callbacks
    # def inversekin(self,msg):
    #   if msg.solved == 1 and self.check == True:
    #      self.invkin.data[0] = msg.q[0]
    #     self.invkin.data[1] = msg.q[1]
    #    self.invkin.data[2] = msg.q[2]
    #   self.invkin.data[3] = msg.q[3]
    #  self.wristangle.data[0] = msg.q[4]
    # self.wristangle.data[1] = msg.q[5]


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
                self.state.mode = 'IK Arm - Base,Tool'
            elif self.state.mode == 'IK Arm - Base,Tool':
                self.state.mode = 'IK Arm - Tool,Tool'
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
    # INVERSE KINEMATICS CONTROL Position = Base Frame; Orientation = End effector frame
    # ==========================================================================
    def arm_IK_base_tool(self):

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
            
        ANGLE_RATE = 5
        
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
        curRot = self.posemsg_to_matrix(self.pose_cmd)
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        
        self.pose_cmd.position.x += axes[0]*MAX_RATE
        self.pose_cmd.position.y -= axes[1]*MAX_RATE
        self.pose_cmd.position.z += axes[4]*MAX_RATE
        alpha = axes[2]*MAX_RATE*ANGLE_RATE
        beta = axes[5]*MAX_RATE*ANGLE_RATE
        gamma = axes[3]*MAX_RATE*ANGLE_RATE
        Rx = tf.transformations.rotation_matrix(alpha, xaxis)
        Ry = tf.transformations.rotation_matrix(beta,  yaxis)
        Rz = tf.transformations.rotation_matrix(gamma, zaxis)
        newRot = tf.transformations.concatenate_matrices(curRot,Rx,Ry,Rz)
        quat = tf.transformations.quaternion_from_matrix(newRot)

        self.pose_cmd.orientation.x = quat[0]
        self.pose_cmd.orientation.y = quat[1]
        self.pose_cmd.orientation.z = quat[2]
        self.pose_cmd.orientation.w = quat[3]
        
       # print self.pose_cmd.position.x, -axes[0]*MAX_RATE, left_joy_right
        
    	# send pose to IK
        self.pub_pose_ik.publish(self.pose_cmd)
        
        #print self.pose_cmd

    def posemsg_to_matrix(self,posemsg):
        quaternion = (
            posemsg.orientation.x,
            posemsg.orientation.y,
            posemsg.orientation.z,
            posemsg.orientation.w)
        H = tf.transformations.quaternion_matrix(quaternion)
        return H

    # ==========================================================================
    # INVERSE KINEMATICS CONTROL 2 Position = End Effector Frame; Orientation = End effector frame
    # ==========================================================================
    def arm_IK_tool_tool(self):

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
            
        ANGLE_RATE = 5
        
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

        ### Update Cartesian Position

        # Get Current Transformation to End Effector Tip
        T = self.posemsg_to_matrix(self.pose_cmd)
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        
        # x,y,z movement in tool frame
        x_mvnt = axes[0]*MAX_RATE
        y_mvnt = -axes[1]*MAX_RATE
        z_mvnt = axes[4]*MAX_RATE
        mvnt = np.matrix([x_mvnt, y_mvnt, z_mvnt])
        dTool = tr.translation_matrix(mvnt)
        dBase = tr.concatenate_matrices(T, dTool)

        self.pose_cmd.position.x = dBase.item(0, 3)
        self.pose_cmd.position.y = dBase.item(1, 3)
        self.pose_cmd.position.z = dBase.item(2, 3)

        alpha = axes[2]*MAX_RATE*ANGLE_RATE
        beta = axes[5]*MAX_RATE*ANGLE_RATE
        gamma = axes[3]*MAX_RATE*ANGLE_RATE
        Rx = tr.rotation_matrix(alpha, xaxis)
        Ry = tr.rotation_matrix(beta,  yaxis)
        Rz = tr.rotation_matrix(gamma, zaxis)
        newRot = tr.concatenate_matrices(T, Rx, Ry, Rz)
        quat = tr.quaternion_from_matrix(newRot)

        self.pose_cmd.orientation.x = quat[0]
        self.pose_cmd.orientation.y = quat[1]
        self.pose_cmd.orientation.z = quat[2]
        self.pose_cmd.orientation.w = quat[3]
        
       # print self.pose_cmd.position.x, -axes[0]*MAX_RATE, left_joy_right
        
        # send pose to IK
        self.pub_pose_ik.publish(self.pose_cmd)
        
        #print self.pose_cmd

    def posemsg_to_matrix(self,posemsg):
        quaternion = (
            posemsg.orientation.x,
            posemsg.orientation.y,
            posemsg.orientation.z,
            posemsg.orientation.w)
        H = tf.transformations.quaternion_matrix(quaternion)
        return H

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
       
        # Send moved angles to IK
        #self.invkin.data[0] = -180/np.pi*((self.cmd.q1-3905)*3*np.pi/2/4092-3*np.pi/4)
        #self.invkin.data[1] = -180/np.pi*((self.cmd.q2-3696)*3*np.pi/4/4092)
        #self.invkin.data[2] = 180/np.pi*((self.cmd.q3-1500)*np.pi/4092-3*np.pi/4)
        #self.invkin.data[3] = 180/np.pi((self.cmd.q4-945)*15*np.pi/4092-15*np.pi/4)
        #self.dyn.data[0]=self.dyn_cmd.data[0]
        #self.dyn.data[1]=self.dyn_cmd.data[1]

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

        #self.cmd.q1 = 1850
        #self.cmd.q2 = 968
        #self.cmd.q3 = 2891
        #self.cmd.q4 = 1968
        #self.cmd.q5 = 0.
        #self.cmd.q6 = 0.0
        
        # set flag so IK knows must init when entered again
        self.init_ik = True

        # Publish arm commands
        self.pub_joints.publish(self.joints)
        self.pub_joint_ik.publish(self.joints)


#     # ==========================================================================
#     # Velocity Arm Control ===============================================
#     # ==========================================================================
#     def vel_cmd(self):
        
#         # Speed Check
#         self.speed_check()

#         # Set corresponding rate
#         if self.state.speed == 'Fast':
#         	MAX_RATE = 100
#         elif self.state.speed == 'Med':
#         	MAX_RATE = 75
#         elif self.state.speed == 'Slow':
#         	MAX_RATE = 50

#         # Calculate how to command arm (position control)
#         DEADZONE = 0.1
        
#         # Set axes
#         left_joy_up = self.joy.axes[1]
#         left_joy_right = self.joy.axes[0]
#         right_joy_up = self.joy.axes[4]
#         right_joy_right = self.joy.axes[3]
#         hat_up = self.joy.axes[7]
#         hat_right = self.joy.axes[6]
        
#         # make array of axes
#         axes = [left_joy_right, left_joy_up, hat_up,
#             hat_right, right_joy_up, right_joy_right]
        
#         # Set axis to zero in deadzone
#         for i in range(0,len(axes)):
#             if abs(axes[i])<DEADZONE:
#                 axes[i] = 0
                
#         # Update joint angles
#         for i in range(0,6):
#             self.joints.velocity[i] = axes[i]*MAX_RATE
        
# #        # Set joint angle limits
# #        for i in range(0,len(self.joints.position)):
# #            if self.joints.position[i] > np.pi:
# #                self.joints.position[i] = np.pi
# #            elif self.joints.position[i] < -np.pi:
# #                self.joints.position[i] = -np.pi

#         self.joints.header.stamp = rospy.Time.now()
#         self.joints.header.frame_id = 'VelControl'        

#         # Publish arm commands
#         self.pub_joints.publish(self.joints)

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
	            elif xbox.state.mode == 'IK Arm - Base,Tool':
	                xbox.arm_IK_base_tool()
	            elif xbox.state.mode == 'IK Arm - Tool,Tool':
	                xbox.arm_IK_tool_tool()
	            else:
	                xbox.joint_cmd()
        #else:
         #   xbox.pub_joints.publish(xbox.joints)

        rate.sleep()

