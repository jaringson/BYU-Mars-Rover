#!/usr/bin/env python
import serial
import time
import rospy
import ast
import math
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import scipy.io as sio

class IMUlive():

    def __init__(self,usb_port_number):
        self.num = str(usb_port_number)
        # In order to get access to /dev/ttyUSB0 1 & 2 you will need to do the following:
        # execute sudo adduser $USER dialout and logout and in again or
        #sudo chmod 666 /dev/ttyUSB0 and 1 and 2 (3 commands) - This is for a single session only
        self.reader=serial.Serial('/dev/ttyUSB'+self.num,250000,timeout=1)
        self.reader.flush()
        trash=self.reader.readline()
        print trash

        self.gyro_offset = [0,0,0]
        self.accel_offset = 1

        # ROS messages
        self.msg_accel_gyro = Imu()
        self.gyro = self.msg_accel_gyro.angular_velocity
        self.accel = self.msg_accel_gyro.linear_acceleration
        self.msg_mag = Vector3Stamped()
        self.mag = self.msg_mag.vector

        # Set up ros publisher
        rospy.init_node('IMU2serial')
        self.accel_gyro_pub = rospy.Publisher('imu_Ada_'+self.num+'/data_raw',Imu, queue_size=1)
        self.mag_pub = rospy.Publisher('imu_Ada_'+self.num+'/mag',Vector3Stamped, queue_size=1)

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

    
        # run calibrator
        yep = raw_input('Do you wish to calibrate imu '+self.num+'? (y/n): ')
        if yep == 'y':
            self.calibrated = self.calibrate()
        else:
            parameters = sio.loadmat('imu'+self.num+'calibration.mat')
            self.magx_range = parameters['magx_range']
            self.magx_mid = parameters['magx_mid']
            self.magy_range = parameters['magy_range']
            self.magy_mid = parameters['magy_mid']
            self.magz_range = parameters['magz_range']
            self.magz_mid = parameters['magz_mid']

    def calibrate(self):
        print "Shake the magnetometer around for 30 seconds"
        start_time = time.time()
        while time.time()-start_time <30:
        	
            self.get_data()
            if self.first==1:
                self.magx_max = self.mag.x
                self.magy_max = self.mag.y
                self.magz_max = self.mag.z
                self.magx_min = self.mag.x
                self.magy_min = self.mag.y
                self.magz_min = self.mag.z
                self.first = 0
            if self.mag.x > self.magx_max and self.mag.x !=0:
                self.magx_max = self.mag.x
            elif self.mag.x < self.magx_min and self.mag.x !=0:
                self.magx_min = self.mag.x

            if self.mag.y > self.magy_max and self.mag.y !=0:
                self.magy_max = self.mag.y
            elif self.mag.y < self.magy_min and self.mag.y !=0:
                self.magy_min = self.mag.y

            if self.mag.z > self.magz_max and self.mag.z !=0:
                self.magz_max = self.mag.z
            elif self.mag.z < self.magz_min and self.mag.z !=0:
                self.magz_min = self.mag.z

        self.magx_range = self.magx_max-self.magx_min
        self.magy_range = self.magy_max-self.magy_min
        self.magz_range = self.magz_max-self.magz_min

        self.magx_mid = self.magx_min + self.magx_range/2.0
        self.magy_mid = self.magy_min + self.magy_range/2.0
        self.magz_mid = self.magz_min + self.magz_range/2.0

        parameters = {'magx_range':self.magx_range,
                      'magx_mid':self.magx_mid,
                      'magy_range':self.magy_range,
                      'magy_mid':self.magy_mid,
                      'magz_range':self.magz_range,
                      'magz_mid':self.magz_mid}

        sio.savemat('imu'+self.num+'calibration.mat',parameters)
                      
        
        return 1

    def get_data(self):
        
        # publish headers
        self.msg_accel_gyro.header.stamp = rospy.Time().now()
        self.msg_accel_gyro.header.frame_id = "imu_board_Ada_"+self.num
        self.msg_mag.header.stamp = rospy.Time().now()
        self.msg_mag.header.frame_id = "imu_board_Ada_"+self.num

        
        # read in new line of data
        start = time.time()
        self.buf_var = self.reader.readline() #This takes .007 s to execute
        finish = time.time()        
        #print self.buf_var

        # assign data to messages

        try:
            if self.calibrated == 1:
                self.buf_list = ast.literal_eval(self.buf_var)
                self.gyro.x = self.buf_list[0]-self.gyro_offset[0]
                self.gyro.y = self.buf_list[1]-self.gyro_offset[1]
                self.gyro.z = self.buf_list[2]-self.gyro_offset[2]
                self.accel.x = self.buf_list[3]*self.accel_offset
                self.accel.y = self.buf_list[4]*self.accel_offset
                self.accel.z = self.buf_list[5]*self.accel_offset
                self.mag.x = (self.buf_list[6]-self.magx_mid)*20.0/self.magx_range
                self.mag.y = (self.buf_list[7]-self.magy_mid)*20.0/self.magy_range
                self.mag.z = (self.buf_list[8]-self.magz_mid)*20.0/self.magz_range
            else:
                self.buf_list = ast.literal_eval(self.buf_var)
                self.gyro.x = self.buf_list[0]-self.gyro_offset[0]
                self.gyro.y = self.buf_list[1]-self.gyro_offset[1]
                self.gyro.z = self.buf_list[2]-self.gyro_offset[2]
                self.accel.x = self.buf_list[3]*self.accel_offset
                self.accel.y = self.buf_list[4]*self.accel_offset
                self.accel.z = self.buf_list[5]*self.accel_offset
                self.mag.x = self.buf_list[6]
                self.mag.y = self.buf_list[7]
                self.mag.z = self.buf_list[8]
        except:
           #print self.buf_list
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
    print "Created IMU instances"
    rospy.sleep(1)
    
    imu_number = raw_input('Do you want to run 3 or 1 IMU(s)? (3/1): ')
    
    if imu_number == "3":
    	Ada0 = IMUlive(0)
    	Ada1 = IMUlive(1)
        Ada2 = IMUlive(2)
    else:
    	Ada0 = IMUlive(0)

    while not rospy.is_shutdown():
        
        if imu_number == "3":
        	Ada0.publish_to_ros()
        	Ada1.publish_to_ros()
        	Ada2.publish_to_ros()
        else:
        	Ada0.publish_to_ros()

        

