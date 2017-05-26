#!/usr/bin/env python

import rospy, math
#from ctypes import c_ushort
from geometry_msgs.msg import Twist
from rover_msgs.msg import RoverState, Drive
from sensor_msgs.msg import Joy, JointState, NavState
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
from wheel_controller import WheelControl
from WP_Publisher import WP_Publisher
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

        # Import wheel controller
        self.wheel_controller = WheelControl()
        self.wheel_controller.enable = False

        # Autonomous stuff
        self.last_NED = [0, 0]
        self.last_gps = [0, 0]
        self.wp_lat = []
        self.wp_lon = []
        self.wp_N = []
        self.wp_E = []
        self.good_est = False
        self.wp_pubs = WP_Publisher()

        # Publishers and Subscribers
        self.sub_joy = rospy.Subscriber('/joy_drive', Joy, self.joyCallback)
        self.sub_gimbal = rospy.Subscriber('/gimbal_feedback', Float32MultiArray, self.gimbalCallback)
        self.sub_estimate = rospy.Subscriber('/estimate', NavState, self.estimateCallback)
        self.pub_drive = rospy.Publisher('/drive_cmd', Drive, queue_size = 1)
        self.pub_state = rospy.Publisher('/rover_state_cmd', RoverState, queue_size = 1)

        self.ready = False

        self.ready_msg = False
        self.trigger_init = {'left': False, 'right': False}


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
                rospy.loginfo('Drive Mode: Drive-Vel')
            elif self.state.mode == 'Drive-Vel':
                self.state.mode = 'Auto'
                rospy.loginfo('Drive Mode: Auto. Hold right trigger to enable')
            else:
                self.zeroSpeed()
                self.state.mode = 'Drive'
                rospy.loginfo('Drive Mode: Drive')
            time.sleep(.25)

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

################################################
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

    def estimateCallback(self, msg):
        self.last_NED = [msg.position[0], msg.position[1]]
        self.last_gps = [msg.gps_pos[0], msg.gps_pos[1]]
        self.good_est = True


##################################################
    def trigger_check(self):
        rt = (1 - self.joy.axes[5])/2.0
        lt = (1 - self.joy.axes[2])/2.0
        if rt == 1:
            self.trigger_init['right'] = True
        if lt == 1:
            self.trigger_init['left'] = True

        if self.trigger_init['left'] and self.trigger_init['right']:
            return True
        else:
            return False

                
#######################
    def speed_check(self):
        # toggle between drive speeds
        rb = self.joy.buttons[5]
        if rb == 1:
            if self.state.speed == 'Slow':
                self.state.speed = 'Med'
                rospy.loginfo('Drive Speed: Medium')
            elif self.state.speed == 'Med':
                self.state.speed = 'Fast'
                rospy.loginfo('Drive Speed: Fast')
            elif self.state.speed == 'Fast':
                self.state.speed = 'Slow'
                rospy.loginfo('Drive Speed: Slow')
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

        # Add waypoint if click lb
        # lb = self.joy.buttons[4]
        if self.joy.buttons[4]: #LB button
            if self.good_est:
                self.wp_lat.append(self.last_gps[0])
                self.wp_lon.append(self.last_gps[1])
                self.wp_N.append(self.last_NED[0])
                self.wp_E.append(self.last_NED[1])
                rospy.logwarn('Added Waypoint To Q!')
                with open('NED_waypoints.txt','a') as myfile:
                    myfile.write(str(self.last_NED[0]))
                    myfile.write(str(self.last_NED[1]))
                    myfile.write('\n')
                # with open('GPS_waypoints.txt','a') as myfile:
                #     myfile.write(str(self.last_gps[0]))
                #     myfile.write(str(self.last_gps[1]))
                #     myfile.write('\n')
            else:
                rospy.logwarn('ERROR: Waypoint not added, no good estimate')
            time.sleep(.25)

        if self.joy.buttons[6]: # Back button
            self.wp_lat = []
            self.wp_lon = []
            self.wp_N = []
            self.wp_E = []
            rospy.logwarn('Waypoint Q reset')
        if (self.joy.axes[2] == -1) and self.joy.buttons[7]: # LT AND START
            # Send waypoints from txt file
            # SEND as NED
            with open('waypoints_to_be_sent_NED.txt') as file:
                for line in f:
                    NED = line.split()
                    self.wp_N.append(NED[0])
                    self.wp_E.append(NED[1])
            self.wp_pubs.SendWaypoints(self.wp_N, self.wp_E)

            # SEND AS gps
            # with open('waypoints_to_be_sent_gps.txt') as file:
            #     for line in f:
            #         latlon = line.split()
            #         self.wp_lat.append(latlon[0])
            #         self.wp_lon.append(latlon[1])
            # self.wp_pubs.convertAndSendWaypoints(self.wp_lat, self.wp_lon)

        if self.joy.buttons[7]: # Start button
            self.wp_pubs.SendWaypoints(self.wp_N, self.wp_E)
            rospy.logwarn('Waypoints sent in NED')
            # self.wp_pubs.convertAndSendWaypoints(self.wp_lat, self.wp_lon)
            # rospy.logwarn('Waypoints send in GPS')
        
    # # ==========================================================================
    # # Velocity Drive Control ===============================================
    # # ==========================================================================
    # def driveVelCommand(self):
    # # Check for slow/medium/fast mode
    #     self.speed_check()

    #     # set joystick commands
    #     right_joy_left = self.joy.axes[3]
    #     right_joy_up = self.joy.axes[4]

    #     # Calculate drive speeds
    #     # rw commands were multiplied by (-1)
    #     msg = Twist()
    #     if self.state.speed == 'Fast': # max = 2000
    #         msg.angular.z = right_joy_left*self.wheel_controller.max_V
    #         msg.linear.x = right_joy_up*self.wheel_controller.max_V 
    #     elif self.state.speed == 'Med': # max = 1750
    #         msg.angular.z = right_joy_left*self.wheel_controller.max_V*0.5
    #         msg.linear.x = right_joy_up*self.wheel_controller.max_V*0.5
    #     elif self.state.speed == 'Slow': # max = 1675
    #         msg.angular.z = right_joy_left*self.wheel_controller.max_V*0.35
    #         msg.linear.x = right_joy_up*self.wheel_controller.max_V*0.35

    #     # Publish Drive Command
    #     self.drive_cmd = self.wheel_controller.Twist2Drive(msg)
    #     self.pub_drive.publish(self.drive_cmd)

    # ==========================================================================
    # Auto Drive Control ===============================================
    # ==========================================================================
    def autoCommand(self):

        rt = (1 - self.joy.axes[5])/2.0
        threshold = 0.1
        if rt > threshold:
            self.driveCommand()
            self.wheel_controller.enable = False
        else:
            self.wheel_controller.enable = True

    def zeroSpeed(self):
        self.drive_cmd.lw = 0
        self.drive_cmd.rw = 0
        self.wheel_controller.enable = False
        self.pub_drive.publish(self.drive_cmd)

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

    rospy.loginfo("Press and release both Triggers to initialize")
    
    # Loop
    while not rospy.is_shutdown():
        # only run when Xbox controller recognized
        if len(xbox.joy.buttons) > 0 and xbox.trigger_check():

            if not xbox.ready_msg:
                rospy.loginfo('Drive Controller Ready')
                xbox.ready_msg = True

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

