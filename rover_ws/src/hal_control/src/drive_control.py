#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
from geometry_msgs.msg import Twist
from rover_msgs.msg import RoverState, Drive
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
from wheel_controller import WheelControl
import time
import numpy as np


class XBOX():

    def __init__(self):
    # Variables
        self.joy = Joy()
        self.state = RoverState()
        self.drive_cmd = Drive()
        
        # Initialize state
        self.state.mode = 'Drive' # Drive, Auto 
        self.state.speed = 'Med' # Slow, Med, Fast
        self.state.kill = False
        self.state.pan = 0.0
        self.state.tilt = 0.0
        self.state.camtoggle1 = False
        self.state.chutes = 67 # All closed
        
        # Initialize Drive
        self.drive_cmd.lw = 0
        self.drive_cmd.rw = 0

    # Publishers and Subscribers
        self.sub_joy = rospy.Subscriber('/joy_drive', Joy, self.joyCallback)
        self.sub_gimbal = rospy.Subscriber('/gimbal_feedback', Float32MultiArray, self.gimbalCallback)
        self.pub_drive = rospy.Publisher('/drive_cmd', Drive, queue_size = 1)
        self.pub_state = rospy.Publisher('/rover_state_cmd', RoverState, queue_size = 1)

        self.ready = False


    # Functions
    
    #################
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes
        home = self.joy.buttons[8]
        # if y == 1: # UNCOMMENT THIS TO SWITCH BETWEEN MODES WITH THE Y BUTTON
        #     if self.state.mode == 'Drive':
        #         self.state.mode = 'Auto'
        #     else:
        #         self.state.mode = 'Drive'
            # time.sleep(.25)

        # Implement Kill Switch
        if home == 1:
            if self.state.kill == False:
                self.state.kill  = True
            else:
                self.state.kill  = False
            time.sleep(.25)

        # check for camera toggle
        self.camera_toggle()

        # check for chutes
        self.chutes()

        # gimbal
        self.cam_pan_tilt()
        
        # Publish state commands
        if self.ready:
            self.pub_state.publish(self.state)

#####################
    def joyCallback(self,msg):
        self.joy=msg
        if self.joy.buttons[9] == 1:
            if self.check==False:            
                self.check=True
            else:
                self.check=False

    def gimbalCallback(self, msg):
        self.gimbal_feedback = msg
        if not self.ready:
            print "Load current angles"
            self.state.pan = msg.data[0]
            self.state.tilt = msg.data[1]
            self.ready = True
            time.sleep(0.1)

                
#######################
    def speed_check(self):
        # toggle between drive speeds
        rb = self.joy.buttons[5]
        if rb == 1:
            if self.state.speed == 'Slow':
                self.state.speed = 'Med'
            elif self.state.speed == 'Med':
                self.state.speed = 'Fast'
            elif self.state.speed == 'Fast':
                self.state.speed = 'Slow'
            time.sleep(.25)

######################
    def camera_toggle(self):
        self.state.camtoggle1 = self.joy.buttons[0]

########################
    def cam_pan_tilt(self):
        hat_x = self.joy.axes[6]
        hat_y = self.joy.axes[7]
        A = self.joy.buttons[0]
        left_trigger = self.joy.axes[2]
        right_trigger = self.joy.axes[5]

        ang_inc = math.radians(1)

        # Pan
        if abs(hat_x) > 0.5:
            self.state.pan += ang_inc*np.sign(hat_x)

        if self.state.pan > math.radians(207):
            self.state.pan = math.radians(207)
        elif self.state.pan < math.radians(-90):
            self.state.pan = math.radians(-90)
    
        # Tilt
        if abs(hat_y) > 0.5:
            self.state.tilt += ang_inc*np.sign(hat_y)

        if self.state.tilt > math.radians(90):
            self.state.tilt = math.radians(90)
        elif self.state.tilt < math.radians(-90):
            self.state.tilt = math.radians(-90)

        if A==1: # home
            self.state.pan = 0.0
            self.state.tilt = 0.0
        if left_trigger < 0.8 and A==1: # chutes
            self.state.pan = 2.705
            self.state.tilt = -1.222
        if right_trigger < 0.8 and A==1: # behind
            self.state.pan = 3.22
            self.state.tilt = -0.4189


    # ==========================================================================
    # Drive Control ===============================================
    # ==========================================================================
    def driveCommand(self):
        # Check for slow/medium/fast mode
        self.speed_check()

        # set joystick commands
        left_joy_up = self.joy.axes[1]
        right_joy_up = self.joy.axes[4]

        # Calculate drive speeds
        # rw commands were multiplied by (-1)
        if self.state.speed == 'Fast': # max = 2000
            self.drive_cmd.lw = left_joy_up*100
            self.drive_cmd.rw = right_joy_up*100 
        elif self.state.speed == 'Med': # max = 1750
            self.drive_cmd.lw = left_joy_up*50
            self.drive_cmd.rw = right_joy_up*50
        elif self.state.speed == 'Slow': # max = 1675
            self.drive_cmd.lw = left_joy_up*35
            self.drive_cmd.rw = right_joy_up*35

        # Publish drive commands
        self.pub_drive.publish(self.drive_cmd)
        
    # ==========================================================================
    # Auto Drive Control ===============================================
    # ==========================================================================
    def autoCommand(self):
    # RIGHT NOW THIS IS A COPY OF Xbox Drive
    
        # Check for slow/medium/fast mode
        self.speed_check()

        # set joystick commands
        left_joy_up = self.joy.axes[1]
        right_joy_up = self.joy.axes[4]

        # Calculate drive speeds
        # rw commands were multiplied by (-1)
        if self.state.speed == 'Fast': # max = 2000
            self.drive_cmd.lw = left_joy_up*100
            self.drive_cmd.rw = right_joy_up*100 
        elif self.state.speed == 'Med': # max = 1750
            self.drive_cmd.lw = left_joy_up*50
            self.drive_cmd.rw = right_joy_up*50
        elif self.state.speed == 'Slow': # max = 1675
            self.drive_cmd.lw = left_joy_up*35
            self.drive_cmd.rw = right_joy_up*35

        # Pan and Tilt
        #self.cam_pan_tilt() # NEED TO IMPLEMENT

        # Turn analog video on or off with left bumper
        # On/off is most significant bit in camnum in command
#        lb = self.joy.buttons[4]
#        if lb == 1:
#            self.analog_cam ^= 1
#            time.sleep(.25)

        # Publish drive commands
        self.pub_drive.publish(self.drive_cmd)
        
    def driveVelCommand(self):
    # Check for slow/medium/fast mode
        self.speed_check()

        # set joystick commands
        right_joy_left = self.joy.axes[3]
        right_joy_up = self.joy.axes[4]

        # Calculate drive speeds
        # rw commands were multiplied by (-1)
        msg = Twist()
        if self.state.speed == 'Fast': # max = 2000
            msg.angular.z = right_joy_left*self.wheel_controller.max_V
            msg.linear.x = right_joy_up*self.wheel_controller.max_V 
        elif self.state.speed == 'Med': # max = 1750
            msg.angular.z = right_joy_left*self.wheel_controller.max_V*0.5
            msg.linear.x = right_joy_up*self.wheel_controller.max_V*0.5
        elif self.state.speed == 'Slow': # max = 1675
            msg.angular.z = right_joy_left*self.wheel_controller.max_V*0.35
            msg.linear.x = right_joy_up*self.wheel_controller.max_V*0.35

        # Publish Drive Command
        self.drive_cmd = self.wheel_controller.Twist2Drive(msg)
        self.pub_drive.publish(self.drive_cmd)

    # ==========================================================================
    # Auto Drive Control ===============================================
    # ==========================================================================
    def autoCommand(self):
    # RIGHT NOW THIS IS A COPY OF Xbox Drive

        rt = (1 - self.joy.axes[5])/2.0
        threshold = 0.1
        if rt > threshold:
            self.wheel_control.enable = False
            self.driveCommand()
        else:
            self.wheel_control.enable = True


    # ==========================================================================
    # Chutes mode ===============================================
    # ==========================================================================
    def chutes(self):
        X = self.joy.buttons[2]
        B = self.joy.buttons[1]

        if X and not B:
            self.state.chutes = 1 #65
        elif B and not X:
            self.state.chutes = 2 #66
        elif X and B:
            self.state.chutes = 3 #64
        else:
            self.state.chutes = 0 #67 

    # ==========================================================================
    # Kill State ===============================================
    # ==========================================================================    

    def killstate(self):
        # Publish zero wheel velocity for kill state
        self.drive_cmd.lw = 0
        self.drive_cmd.rw = 0
        self.pub_drive.publish(self.drive_cmd)

    # ==========================================================================
    # Main ===============================================
    # ==========================================================================
if __name__ == '__main__':
    # init ROS node
    rospy.init_node('xbox_drive_control')

    # set rate
    hz = 60.0
    rate = rospy.Rate(hz)    
    
    # init XBOX object
    xbox = XBOX()
    xbox.ready = True
    
    # Loop
    while not rospy.is_shutdown():
        # only run when Xbox controller recognized
        if len(xbox.joy.buttons) > 0:

            # every time check toggle of state and cameras
            xbox.check_method()

            # check for kill switch (True = Killed)
            if xbox.state.kill == False:

                # call appropriate function for state
                # defaults to 'Drive'
                if xbox.state.mode == 'Drive':
                    xbox.driveCommand()
                elif xbox.state.mode == 'Auto':
                    xbox.autoCommand() # NEED this def
                elif xbox.state.mode == 'Drive-Vel':
                    xbox.driveVelCommand()
                else:
                    xbox.driveCommand()
            else:
                xbox.killstate()

        rate.sleep()

