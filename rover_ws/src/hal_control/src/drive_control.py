#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
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
        self.state.pan = 1500
        self.state.tilt = 1500
        self.state.camtoggle1 = False
        self.state.chutes = 0
        
        # Initialize Drive
        self.drive_cmd.lw = 0
        self.drive_cmd.rw = 0

	# Instantiate Wheel_control class and disable it
	self.wheel_control = WheelControl()
	self.wheel_control.enable = False

    # Publishers and Subscribers
        self.sub_joy = rospy.Subscriber('/joy_drive', Joy, self.joyCallback)
        self.pub_drive = rospy.Publisher('/drive_cmd', Drive, queue_size = 10)
        self.pub_state = rospy.Publisher('/rover_state_cmd', RoverState, queue_size = 10)

    # Functions
    
    #################
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes
        home = self.joy.buttons[8]
        if y == 1: # UNCOMMENT THIS TO SWITCH BETWEEN MODES WITH THE Y BUTTON
            if self.state.mode == 'Drive':
                self.state.mode = 'Auto'
				self.loginfo('Switched to %s Drive State') % self.state.mode
            elif self.state.mode == 'Drive-Vel':
                self.state.mode = 'Auto'
				self.loginfo('Switched to %s Drive State') % self.state.mode
			else:
				self.state.mode = 'Drive'
				self.loginfo('Switched to %s Drive State') % self.state.mode
            time.sleep(.25)

        # Implement Kill Switch
        if home == 1:
            if self.state.kill == False:
                self.state.kill  = True
				rospy.loginfo('Kill Switch Activated')
            else:
                self.state.kill  = False
				rospy.loginfo('Kill Switch Deactivated')
            time.sleep(.25)

        # check for camera toggle
        self.camera_toggle()
        
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
    def camera_toggle(self):

        self.state.camtoggle1 = self.joy.buttons[0]
        if self.state.camtoggle1:
           self.pub_state.publish(self.state)
        else:
          self.pub_state.publish(self.state)


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
				elif xbox.state.mode == 'Drive-Vel':
					xbox.driveVelCommand()
	            else:
	                xbox.driveCommand()

        rate.sleep()

