#!/usr/bin/env python
import time
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import scipy.io as sio

class IMUCalibration():
    def __init__(self,topic):
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.mx = 0
        self.my = 0
        self.mz = 0
        self.first = 1
        rospy.Subscriber("/"+topic+"/data_raw",Imu, self.accel_gyro_callback, tcp_nodelay=True)
        rospy.Subscriber("/"+topic+"/mag",Vector3Stamped, self.mag_callback, tcp_nodelay=True)
        
    def accel_gyro_callback(self,data):
        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z
        self.gx = data.angular_velocity.x
        self.gy = data.angular_velocity.y
        self.gz = data.angular_velocity.z
        
    def mag_callback(self,data):
        self.mx = data.vector.x
        self.my = data.vector.y
        self.mz = data.vector.z
            
                        
    def calibrate(self):
        print "Shake the magnetometer around for 20 seconds"
        rospy.sleep(1)
        start_time = time.time()
        while time.time()-start_time < 20:
            if self.first==1:
                self.ax_max = self.ax
                self.ay_max = self.ay
                self.az_max = self.az
                self.ax_min = self.ax
                self.ay_min = self.ay
                self.az_min = self.az

                self.gx_max = self.gx
                self.gy_max = self.gy
                self.gz_max = self.gz
                self.gx_min = self.gx
                self.gy_min = self.gy
                self.gz_min = self.gz

                self.mx_max = self.mx
                self.my_max = self.my
                self.mz_max = self.mz
                self.mx_min = self.mx
                self.my_min = self.my
                self.mz_min = self.mz

                self.first = 0

            else:

                if self.mx > self.mx_max:
                    self.mx_max = self.mx
                elif self.mx < self.mx_min:
                    self.mx_min = self.mx

                if self.my > self.my_max:
                    self.my_max = self.my
                elif self.my < self.my_min:
                    self.my_min = self.my

                if self.mz > self.mz_max:
                    self.mz_max = self.mz
                elif self.mz < self.mz_min:
                    self.mz_min = self.mz

        self.mx_range = self.mx_max-self.mx_min
        self.my_range = self.my_max-self.my_min
        self.mz_range = self.mz_max-self.mz_min

        self.mx_mid = self.mx_min + self.mx_range/2.0
        self.my_mid = self.my_min + self.my_range/2.0
        self.mz_mid = self.mz_min + self.mz_range/2.0

        mx_offset = self.mx_mid*20.0/self.mx_range
        my_offset = self.my_mid*20.0/self.my_range
        mz_offset = self.mz_mid*20.0/self.mz_range

        mx_scale = 20.0/self.mx_range
        my_scale = 20.0/self.my_range
        mz_scale = 20.0/self.mz_range

        parameters = {"mx_offset":mx_offset,
                      "my_offset":my_offset,
                      "mz_offset":mz_offset,
                      "mx_scale":mx_scale,
                      "my_scale":my_scale,
                      "mz_scale":mz_scale}

        print parameters

        sio.savemat(topic+'calibration.mat',parameters)

if __name__=='__main__':
    rospy.init_node('Calibrator')
    while not rospy.is_shutdown():
        topic = raw_input("which imu do you want to calibrate?")
        calib = IMUCalibration(topic)
        calib.calibrate()
    
