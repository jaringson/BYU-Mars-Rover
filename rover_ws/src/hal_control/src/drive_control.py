#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
from rover_msgs.msg import RoverState, Drive
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
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
        self.state.pan = 1500
        self.state.tilt = 1500
        self.state.camnum = 0
        self.state.chutes = 0
        
        # Initialize Drive
        self.drive_cmd.lw = 1500
        self.drive_cmd.rw = 1500

    # Publishers and Subscribers
        self.sub_joy = rospy.Subscriber('/joy_drive', Joy, self.joyCallback)
        self.pub_drive = rospy.Publisher('/rover_drive', Drive, queue_size = 10)
        self.pub_state = rospy.Publisher('/rover_state', RoverState, queue_size = 10)
        # self.pub_cam = rospy.Publisher('/camera_state', camera_select, queue_size = 10)

    # Functions
    
    #################3
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes
        home = self.joy.buttons[8]
        if y == 1:
            if self.state.mode == 'Drive':
                self.state.mode = 'Auto'
            else:
                self.state.mode = 'Drive'
            time.sleep(.25)

        # Implement Kill Switch
        if home == 1:
            if self.state.kill == False:
                self.state.kill  = True
            else:
                self.state.kill  = False
            time.sleep(.25)

        # check for camera toggle
        #self.camera_select() # NEED TO IMPLEMENT
        
         # Publish state commands
        self.pub_state.publish(self.state)

#####################
    def joyCallback(self,msg):
        self.joy=msg
        if self.joy.buttons[9] == 1:
            if self.check==False:            
                self.check=True
            else:
                self.check=False
                
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
    def camera_select(self):

        a = self.joy.buttons[0]


#        # a selects between cameras 0-2, b selects between cameras 3-5
#        # cam1_sel is lower nybble, cam2_sel is upper nybble
#        a = self.joy.buttons[0]
#        b = self.joy.buttons[1]

#        if a == 1:
#            if self.cam1_sel == 2:
#                self.cam1_sel = 0
#            else:
#                self.cam1_sel = self.cam1_sel + 1
#            time.sleep(.25)
#        if b == 1:
#            if self.cam2_sel == 2:
#                self.cam2_sel = 0
#            else:
#                self.cam2_sel = self.cam2_sel + 1
#            time.sleep(.25)
#        # Update command
#        self.state.camnum = (self.analog_cam << 7) | ((self.cam1_sel & 0x0f) | ((self.cam2_sel & 0x0f) << 4))

########################
#    def cam_pan_tilt(self):
#        x = self.joy.buttons[2]
#        back = self.joy.buttons[6]
#        start = self.joy.buttons[7]
#        push_right = self.joy.buttons[10]
#        push_left = self.joy.buttons[9]

#        if x == 1:
#            self.cmd.pan = 1500
#            self.cmd.tilt = 1500
#            time.sleep(.05)
#        if start == 1:
#            self.cmd.tilt = self.cmd.tilt + 10.0
#            time.sleep(.05)
#        if back == 1:
#            self.cmd.tilt = self.cmd.tilt - 10.0
#            time.sleep(.05)
#        if push_right == 1:
#            self.cmd.pan = self.cmd.pan + 10.0
#            time.sleep(.05)
#        if push_left == 1:
#            self.cmd.pan = self.cmd.pan - 10.0
#            time.sleep(.05)
#        # bounds check
#        if self.cmd.tilt > 2000:
#            self.cmd.tilt = 2000
#        if self.cmd.tilt < 1000:
#            self.cmd.tilt = 1000
#        if self.cmd.pan > 2000:
#            self.cmd.pan = 2000
#        if self.cmd.pan < 1000:
#            self.cmd.pan = 1000

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
        if self.state.speed == 'Fast':
            self.drive_cmd.lw = left_joy_up*500 + 1500
            self.drive_cmd.rw = right_joy_up*500 + 1500
        elif self.state.speed == 'Med':
            self.drive_cmd.lw = left_joy_up*250 + 1500
            self.drive_cmd.rw = right_joy_up*250 + 1500
        elif self.state.speed == 'Slow':
            self.drive_cmd.lw = left_joy_up*175 + 1500
            self.drive_cmd.rw = right_joy_up*175 + 1500

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
        if self.state.speed == 'Fast':
            self.drive_cmd.lw = left_joy_up*500 + 1500
            self.drive_cmd.rw = right_joy_up*-500 + 1500
        elif self.state.speed == 'Med':
            self.drive_cmd.lw = left_joy_up*250 + 1500
            self.drive_cmd.rw = right_joy_up*-250 + 1500
        elif self.state.speed == 'Slow':
            self.drive_cmd.lw = left_joy_up*175 + 1500
            self.drive_cmd.rw = right_joy_up*-175 + 1500

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


    # ==========================================================================
    # Chutes mode ===============================================
    # ==========================================================================
    def chutes(self):
        # 7th bit is enable bit - keep it on
        self.cmd.chutes |= 2^6
        # get chute commands
        c1 = self.joy.buttons[1]
        c2 = self.joy.buttons[2]
        c3 = self.joy.buttons[7]
        c4 = self.joy.buttons[6]
        c5 = self.joy.buttons[5]
        c6 = self.joy.buttons[4]
        
        # toggle whichever chute button was pressed
        if c1 == 1 or c2 == 1 or c3 == 1 or c4 == 1 or c5 == 1 or c6 == 1:
            self.cmd.chutes ^= c1 | (c2 << 1) | (c3 << 2) | (c4 << 3) | (c5 << 4) | (c6 << 5)
            time.sleep(.25)

        # self.cmd.chutes |= self.joy.buttons[1] | (self.joy.buttons[2] << 1) | (self.joy.buttons[7] << 2) | (self.joy.buttons[6] << 3) | (self.joy.buttons[5] << 4) | (self.joy.buttons[4] << 5) | (1 << 6)
        self.pub1.publish(self.cmd)

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
	            else:
	                xbox.driveCommand()

        rate.sleep()

