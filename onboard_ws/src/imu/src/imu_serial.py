#!/usr/bin/env python
import serial
import time
import rospy
import ast
import math
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
import scipy.io as sio
import pylab as p
import mpl_toolkits.mplot3d.axes3d as p3

class IMUlive():

    def __init__(self,usb_port_number):
        self.num = str(usb_port_number)
        # In order to get access to /dev/ttyUSB0 1 & 2 you will need to do the following:
        # execute sudo adduser $USER dialout and logout and in again or
        #sudo chmod 666 /dev/ttyUSB0 and 1 and 2 (3 commands) - This is for a single session only
        self.reader=serial.Serial('/dev/ttyACM0',250000,timeout=1)
       # self.reader=serial.Serial('/dev/ttyUSB0',115200,timeout=1)
        self.reader.flush()
        trash=self.reader.readline()
        print trash

        self.gyro_offset = [-0.01,0.003,0.001]
        self.accel_offset = 1

        # ROS messages
        self.msg_accel_gyro = Imu()
        self.gyro = self.msg_accel_gyro.angular_velocity
        self.accel = self.msg_accel_gyro.linear_acceleration
        self.msg_mag = MagneticField()
        self.mag = self.msg_mag.magnetic_field

        # Set up ros publisher
        rospy.init_node('IMU2serial')
        self.accel_gyro_pub = rospy.Publisher('imu/data_raw',Imu, queue_size=1)
        self.mag_pub = rospy.Publisher('imu/mag',MagneticField, queue_size=1)

        # Initialize data
        self.buf_var = None

        # Magnetometer Calibration variables
        self.first = 1
        self.calibrated = 0
        self.magx_max = 0
        self.magy_max = 0
        self.magz_max = 0
        self.magx_min = 0
        self.magy_min = 0
        self.magz_min = 0
        self.magx_range = 0
        self.magy_range = 0
        self.magz_range = 0
        self.magx_mid = 0
        self.magy_mid = 0
        self.magz_mid = 0

        parameters = sio.loadmat('/home/ubuntu/BYU-Mars-Rover/onboard_ws/src/imu/src/imu'+self.num+'calibration.mat')
        self.magx_range = parameters['magx_range']
        self.magx_mid = parameters['magx_mid']
        self.magy_range = parameters['magy_range']
        self.magy_mid = parameters['magy_mid']
        self.magz_range = parameters['magz_range']
        self.magz_mid = parameters['magz_mid']
        self.calibrated = 1


    
    def get_data(self):

        # publish headers
        self.msg_accel_gyro.header.stamp = rospy.Time().now()
        self.msg_accel_gyro.header.frame_id = "imu_board_Dro_"+self.num
        self.msg_mag.header.stamp = rospy.Time().now()
        self.msg_mag.header.frame_id = "imu_board_Dro_"+self.num


        # read in new line of data
        # start = time.time()
        self.buf_var = self.reader.readline() #This takes .007 s to execute
#	print 'here'
#	print self.buf_var
	   # finish = time.time()
        #print self.buf_var

        # assign data to messages

        try:
            if self.calibrated == 1:
                self.buf_list = ast.literal_eval(self.buf_var)
                self.gyro.x =( self.buf_list[4]-self.gyro_offset[0])*np.pi/180.0
                self.gyro.y = (self.buf_list[5]-self.gyro_offset[1])*np.pi/180.0
                self.gyro.z = (self.buf_list[6]-self.gyro_offset[2])*np.pi/180.0
                self.accel.x = self.buf_list[1]*self.accel_offset
                self.accel.y = self.buf_list[2]*self.accel_offset
                self.accel.z = self.buf_list[3]*self.accel_offset
                # magnitude = np.sqrt(self.accel.x**2 + self.accel.y**2 + self.accel.z**2)
                # self.accel.x = self.accel.x/magnitude
                # self.accel.y = self.accel.y/magnitude
                # self.accel.z = self.accel.z/magnitude
                self.mag.x = (self.buf_list[7]-self.magx_mid)/self.magx_range
                self.mag.y = (self.buf_list[8]-self.magy_mid)/self.magy_range
                self.mag.z = (self.buf_list[9]-self.magz_mid)/self.magz_range
                self.mag.z = -self.mag.z
                x = self.mag.y
                self.mag.y = self.mag.x
                self.mag.x = x
            else:
                print "Error Not Calibrated!"
                self.buf_list = ast.literal_eval(self.buf_var)
                self.gyro.x = (self.buf_list[4]-self.gyro_offset[0])*np.pi/180.0
                self.gyro.y = (self.buf_list[5]-self.gyro_offset[1])*np.pi/180.0
                self.gyro.z = (self.buf_list[6]-self.gyro_offset[2])*np.pi/180.0
                self.accel.x = self.buf_list[1]*self.accel_offset
                self.accel.y = self.buf_list[2]*self.accel_offset
                self.accel.z = self.buf_list[3]*self.accel_offset
                self.mag.x = self.buf_list[7]
                self.mag.y = self.buf_list[8]
                self.mag.z = self.buf_list[9]
        except:
            print self.buf_list
            print "Didn't get that Data\n"
            time.sleep(1/10000)
            self.get_data()


    def publish_to_ros(self):

        rate  = rospy.Rate(2000)
        self.get_data()
        self.accel_gyro_pub.publish(self.msg_accel_gyro)
        self.mag_pub.publish(self.msg_mag)
        rate.sleep()

if __name__=='__main__':

    Dro0 = IMUlive(0)
    #Dro1 = IMUlive(1)
    #Dro2 = IMUlive(2)
    print "Created IMU instances"
    rospy.sleep(1)

    while not rospy.is_shutdown():

        Dro0.publish_to_ros()
