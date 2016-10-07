#!/usr/bin/env python

import rospy, math
from ctypes import c_ushort
from rover_msgs.msg import ArmState
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
import time
import numpy as np


class Arm_XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
        self.state = ArmState()
        self.joints = JointState()
        self.grip = 0
        
        # Initialize state
        self.state.mode = 'JointControl' # Arm, JointControl
        self.state.speed = 'Medium' # Slow, Medium, Fast

        # Initialize joints
        self.joints.header = Header()
        self.joints.header.stamp = rospy.Time.now()
        self.joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints.velocity = []
        self.joints.effort = []      


    # Publishers and Subscribers
        self.sub_joy = rospy.Subscriber('/joy_arm', Joy, self.joyCallback)
        self.pub_state = rospy.Publisher('/arm_state', ArmState, queue_size = 10)
        self.pub_joints = rospy.Publisher('/joint_states', JointState, queue_size = 10)
        self.pub_grip = rospy.Publisher('/grip', Int8, queue_size = 10)
       

    # Callbacks
    #def inversekin(self,msg):
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

    # Functions
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes
        home = self.joy.buttons[8]
        
        if y == 1:
            if self.state.mode == 'JointControl':
                self.state.mode = 'Arm'
            else:
                self.state.mode = 'JointControl'
            time.sleep(.25)
            
        elif home == 1:
            # Implement Kill Switch
            time.sleep(.25)


    def speed_check(self):
        rb = self.joy.buttons[5]
        if rb == 1:
            if self.state.speed == 'Fast':
                self.state.speed = 'Med'
            elif self.state.speed == 'Med':
                self.state.speed = 'Slow'
            elif self.state.speed == 'Slow':
                self.state.speed = 'Fast'
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

        self.cmd.q1=int(round(-196+(self.invkin.data[0]*np.pi/180.0+3.0*np.pi/4.0)*(4092/(3*np.pi/2))))
        self.cmd.q2=int(round(3696+(-self.invkin.data[1]*np.pi/180)*(4092/(3*np.pi/4))))
        self.cmd.q3=int(round(-2224+(-self.invkin.data[2]*np.pi/180+3*np.pi/4)*(4092/(np.pi))))
        #self.cmd.q4=int(round(945+(self.invkin.data[3]*np.pi/180+15*np.pi/4)*(4092/(15*np.pi))))
        
        # make sure they are valid joint angles between [0, 4095]
        # turret
        if self.cmd.q1 < 0:
            self.cmd.q1 = 0
        elif self.cmd.q1 > 4095:
            self.cmd.q1 = 4095
        # shoulder
        if self.cmd.q2 < 0:
            self.cmd.q2 = 0
        elif self.cmd.q2 > 4095:
            self.cmd.q2 = 4095
        # elbow
        if self.cmd.q3 < 0:
            self.cmd.q3 = 0
        elif self.cmd.q3 > 4095:
            self.cmd.q3 = 4095

        '''
        # forearm
        if self.cmd.q4 < 0:
            self.cmd.q4 = 0
        elif self.cmd.q4 > 4095:
            self.cmd.q4 = 4095

        # wrist tilt
        if self.wristangle.data[0]>90.0:
            self.wristangle.data[0]=90.0
        if self.wristangle.data[0]<-90.0:
            self.wristangle.data[0]=-90.0
        # wrist rotate
        if self.wristangle.data[1]>180.0:
            self.wristangle.data[1]=180.0
        if self.wristangle.data[1]<-180.0:
            self.wristangle.data[1]=-180.0
        # set wrist publisher data
        self.dyn_cmd.data[0]=math.radians(self.wristangle.data[0])
        self.dyn_cmd.data[1]=math.radians(self.wristangle.data[1])
        '''

        # Select between camera feeds with A & B on the xbox controller
        self.camera_select()
        
        # Pan and Tilt
        self.cam_pan_tilt()

        # Gripper
        self.gripper()

        # Shovel
        if self.joy.axes[2] < 0:
            self.cmd.shovel = self.cmd.shovel-10.0
            if self.cmd.shovel < 1000:
                self.cmd.shovel = 1000
        elif self.joy.axes[5] < 0:
            self.cmd.shovel = self.cmd.shovel+10.0
            if self.cmd.shovel > 2000:
                self.cmd.shovel = 2000

        # Publish arm commands
        self.pub1.publish(self.cmd)
        #self.pub4.publish(self.dyn_cmd)

    # ==========================================================================
    # Xbox Arm Control ===============================================
    # ==========================================================================
    def joint_cmd(self):
        
        # Speed Check
        self.speed_check()
        
        # Pan and Tilt
        self.cam_pan_tilt()

        # Calculate how to command arm (position control)
        MAX_RATE = 10*np.pi/180
        DEADZONE = 0.1
        
        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]
        
        # Read in axis values
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
       
        # Send moved angles to IK
        #self.invkin.data[0] = -180/np.pi*((self.cmd.q1-3905)*3*np.pi/2/4092-3*np.pi/4)
        #self.invkin.data[1] = -180/np.pi*((self.cmd.q2-3696)*3*np.pi/4/4092)
        #self.invkin.data[2] = 180/np.pi*((self.cmd.q3-1500)*np.pi/4092-3*np.pi/4)
        #self.invkin.data[3] = 180/np.pi((self.cmd.q4-945)*15*np.pi/4092-15*np.pi/4)
        #self.dyn.data[0]=self.dyn_cmd.data[0]
        #self.dyn.data[1]=self.dyn_cmd.data[1]

        # Gripper
        self.gripper()

        # Shovel
        if self.joy.axes[2] < 0:
            self.cmd.shovel = self.cmd.shovel-10.0
            if self.cmd.shovel < 1000:
                self.cmd.shovel = 1000
        elif self.joy.axes[5] < 0:
            self.cmd.shovel = self.cmd.shovel+10.0
            if self.cmd.shovel > 2000:
                self.cmd.shovel = 2000

        #self.cmd.q1 = 1850
        #self.cmd.q2 = 968
        #self.cmd.q3 = 2891
        #self.cmd.q4 = 1968
        #self.cmd.q5 = 0.
        #self.cmd.q6 = 0.0
        
        # Publish arm commands
        self.pub_joints.publish(self.joints)

    # ==========================================================================
    # Main ===============================================
    # ==========================================================================
if __name__ == '__main__':
    rospy.init_node('xbox_control')
    hz = 60.0
    rate = rospy.Rate(hz)
    xbox=Arm_XBOX()
    
    while not rospy.is_shutdown():

        if len(xbox.joy.buttons) > 0:
            # every time check toggle of state and cameras
            xbox.check_method()

            if xbox.state.mode == 'JointControl':
                xbox.joint_cmd()
            elif xbox.state.mode == 'Arm':
                xbox.arm_IK()
            else:
                xbox.joint_cmd()
        else:
            xbox.pub_joints.publish(xbox.joints)

        rate.sleep()

