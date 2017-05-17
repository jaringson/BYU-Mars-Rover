#!/usr/bin/env python
import serial
import time
import rospy
import ast
import math
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray

class IMUlive():

    def __init__(self,usb_port_number):
        self.num = str(usb_port_number)
        # In order to get access to /dev/ttyUSB0 1 & 2 you will need to do the following:
        # execute sudo adduser $USER dialout and logout and in again or
        #sudo chmod 666 /dev/ttyUSB0 and 1 and 2 (3 commands) - This is for a single session only
        self.reader=serial.Serial('/dev/ttyACM1',115200,timeout=1)
       # self.reader=serial.Serial('/dev/ttyUSB0',115200,timeout=1)
        self.reader.flush()
        trash=self.reader.readline()
        print trash

        # ROS messages
        self.angles = Float32MultiArray()

        # Set up ros publisher
        rospy.init_node('IMU2serial')
        self.est_pub = rospy.Publisher('/razorIMU',Float32MultiArray, queue_size=1)

    def get_data(self):

        if self.reader.inWaiting() > 0:
#           print "READ FEEDBACK!"
            self.buf_var = self.reader.readline()
                # if self.ser.read(1).encode('hex') in 'e3':

        # # read in new line of data
        # self.reader.flush()
        # self.buf_var = self.reader.readline() #This takes .007 s to execute
	# print 'here'
	# print self.buf_var
        #print self.buf_var

        # assign data to messages

        try:
            self.buf_list = ast.literal_eval(self.buf_var)
            self.pitch = self.buf_list[0]
            self.roll= self.buf_list[1]
            self.yaw = self.buf_list[2]
            print self.yaw

        except:
            print self.buf_list
            print "Didn't get that Data\n"
            time.sleep(1/10000)
            self.get_data()


    def publish_to_ros(self):

        while not rospy.is_shutdown():
            self.get_data()
            self.angles.data = [self.roll, self.pitch, self.yaw]
            self.est_pub.publish(self.angles)
    

if __name__=='__main__':

    Dro0 = IMUlive(0)
    #Dro1 = IMUlive(1)
    #Dro2 = IMUlive(2)
    print "Created IMU instances"
    rospy.sleep(1)

    while not rospy.is_shutdown():

        Dro0.publish_to_ros()
