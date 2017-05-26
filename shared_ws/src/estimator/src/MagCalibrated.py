#!/usr/bin/env python
import time
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped
import scipy.io as sio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from sensor_msgs.msg import Imu


class MagCalibrated():

    def __init__(self):

        input_topic = rospy.get_param(rospy.get_namespace()+'/input_topic','imu_0/mag_wrong')
        output_topic = rospy.get_param(rospy.get_namespace()+'/output_topic','imu_0/mag')
        self.num = rospy.get_param(rospy.get_namespace()+'/imu_number','0')
        self.run_calibration = rospy.get_param(rospy.get_namespace()+'/run_calibration',0)

        node_name = output_topic.replace('/','_')
        rospy.init_node(node_name)

        # ROS messages
        self.msg_mag = Vector3Stamped()
        self.mag = self.msg_mag.vector

        # Magnetometer Calibration variables
        self.first = 1
        self.calibrated = 0
        self.mag_points = []
        self.mag_points_plot = []
        self.subsample = 0
        self.xmaxcount = 0
        self.ymaxcount = 0
        self.zmaxcount = 0
        self.xmincount = 0
        self.ymincount = 0
        self.zmincount = 0

        if self.run_calibration == 1:
            self.calibrated = self.calibrate()
        else:
            try:
                
                parameters = sio.loadmat(os.path.expanduser('~') + '/git/byu/development/kl_ws/src/low_level_control/src/imu'+self.num+'calibration.mat')
                self.magx_max = parameters['magx_max']
                self.magx_min = parameters['magx_min']
                self.magy_max = parameters['magy_max']
                self.magy_min = parameters['magy_min']
                self.magz_max = parameters['magz_max']
                self.magz_min = parameters['magz_min']
                
                self.calibrated = 1
                print "Loaded saved calibration for IMU ",self.num
            except IOError:
                print "No saved parameters found for IMU ,",self.num,"!"
                self.magx_max = 10
                self.magy_max = 10
                self.magz_max = 10
                self.magx_min = -10
                self.magy_min = -10
                self.magz_min = -10
                self.magx_range =20
                self.magy_range =20
                self.magz_range =20
                self.magx_mid = 0
                self.magy_mid = 0
                self.magz_mid = 0

        self.mag_pub = rospy.Publisher(output_topic,Vector3Stamped,queue_size=0)
        self.magviz_pub = rospy.Publisher(output_topic+'viz',Imu,queue_size=0)        
        rospy.sleep(3)
        self.sub = rospy.Subscriber(input_topic,Vector3Stamped,self.continuous_calibrate_cb)


    def calibrate(self):
        print "Shake magnetometer ",self.num," around for 30 seconds"
        start_time = time.time()
        while time.time()-start_time <30:
            if self.subsample == 250:
                self.mag_points_plot.append([self.mag.x, self.mag.y, self.mag.z])
                self.subsample = 0
            else:
                self.subsample = self.subsample+1
            self.mag_points.append([self.mag.x, self.mag.y, self.mag.z])


        # Find the max, min and middle values
        self.mag_points = np.array(self.mag_points)
        self.magx_max = np.max(self.mag_points[:,0])
        self.magx_min = np.min(slf.mag_points[:,0])
        self.magy_max = np.max(self.mag_points[:,1])
        self.magy_min = np.min(self.mag_points[:,1])
        self.magz_max = np.max(self.mag_points[:,2])
        self.magz_min = np.min(self.mag_points[:,2])


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
                      'magz_mid':self.magz_mid,
                      'magpoints':self.mag_points}

        sio.savemat('imu'+self.num+'calibration.mat',parameters)
        print "Done collecting data"
        print "Plotting data"
        '''
        points = np.array(self.mag_points_plot)
        fig2 = plt.figure(2)
        ax = Axes3D(fig2)
        ax.scatter(points[:,0],points[:,1],points[:,2])        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()
        '''

        print parameters
        print "done calibrating imu ",self.num
                      
        return 1

    def continuous_calibrate_cb(self,msg):
        self.msg_mag.header.stamp = rospy.Time().now()
        self.msg_mag.header.frame_id = "imu_calib_board_"+self.num

        # Update the maximums if necessary
        # X
        if msg.vector.x > self.magx_max and self.xmaxcount > 15:
            print "updated x max on imu ", self.num
            self.magx_max = self.magx_max+.1
            self.xmaxcount = 0
        elif msg.vector.x > self.magx_max and self.xmaxcount <= 15:
            self.xmaxcount = self.xmaxcount+1
        else:
            self.xmaxcount = 0
        # Y
        if msg.vector.y > self.magy_max and self.ymaxcount > 15:
            print "updated y max on imu ", self.num
            self.magy_max = self.magy_max+.1
            self.ymaxcount = 0
        elif msg.vector.y > self.magy_max and self.ymaxcount <= 15:
            self.ymaxcount = self.ymaxcount+1
        else:
            self.ymaxcount = 0
        # Z
        if msg.vector.z > self.magz_max and self.zmaxcount > 15:
            print "updated z max on imu ", self.num
            self.magz_max = self.magz_max+.1
            self.zmaxcount = 0
        elif msg.vector.z > self.magz_max and self.zmaxcount <= 15:
            self.zmaxcount = self.zmaxcount+1
        else:
            self.zmaxcount = 0

        # Update the minimums if necessary
        # X
        if msg.vector.x < self.magx_min and self.xmincount > 15:
            print "updated x min on imu ", self.num
            self.magx_min = self.magx_min-.1
            self.xmincount = 0
        elif msg.vector.x < self.magx_min and self.xmincount <= 15:
            self.xmincount = self.xmincount+1
        else:
            self.xmincount = 0
        # Y
        if msg.vector.y < self.magy_min and self.ymincount > 15:
            print "updated y min on imu ", self.num
            self.magy_min = self.magy_min-.1
            self.ymincount = 0
        elif msg.vector.y < self.magy_min and self.ymincount <= 15:
            self.ymincount = self.ymincount+1
        else:
            self.ymincount = 0
        # Z
        if msg.vector.z < self.magz_min and self.zmincount > 15:
            print "updated z min on imu ", self.num
            self.magz_min = self.magz_min-.1
            self.zmincount = 0
        elif msg.vector.z < self.magz_min and self.zmincount <= 15:
            self.zmincount = self.zmincount+1
        else:
            self.zmincount = 0

        self.magx_range = self.magx_max-self.magx_min
        self.magy_range = self.magy_max-self.magy_min
        self.magz_range = self.magz_max-self.magz_min

        self.magx_mid = self.magx_min + self.magx_range/2.0
        self.magy_mid = self.magy_min + self.magy_range/2.0
        self.magz_mid = self.magz_min + self.magz_range/2.0

        self.mag.x = (msg.vector.x-self.magx_mid)*200.0/self.magx_range
        self.mag.y = (msg.vector.y-self.magy_mid)*200.0/self.magy_range
        self.mag.z = (msg.vector.z-self.magz_mid)*200.0/self.magz_range
        
        vizmsg = Imu()
        vizmsg.header = msg.header
        vizmsg.linear_acceleration.x = self.mag.x
        vizmsg.linear_acceleration.y = self.mag.y
        vizmsg.linear_acceleration.z = self.mag.z

        self.mag_pub.publish(self.msg_mag)
        self.magviz_pub.publish(vizmsg)

    def mag_cb(self,msg):
        self.msg_mag.header.stamp = rospy.Time().now()
        self.msg_mag.header.frame_id = "imu_calib_board_"+self.num

        if self.calibrated == 1:
            self.mag.x = (msg.vector.x-self.magx_mid)*200.0/self.magx_range
            self.mag.y = (msg.vector.y-self.magy_mid)*200.0/self.magy_range
            self.mag.z = (msg.vector.z-self.magz_mid)*200.0/self.magz_range

            #No scaling
            #self.mag.x = (msg.vector.x-self.magx_mid)
            #self.mag.y = (msg.vector.y-self.magy_mid)
            #self.mag.z = (msg.vector.z-self.magz_mid)

            vizmsg = Imu()
            vizmsg.header = msg.header
            vizmsg.linear_acceleration.x = self.mag.x
            vizmsg.linear_acceleration.y = self.mag.y
            vizmsg.linear_acceleration.z = self.mag.z

        else:
            self.mag.x = msg.vector.x
            self.mag.y = msg.vector.y
            self.mag.z = msg.vector.z

            vizmsg = Imu()
            vizmsg.header = msg.header
            vizmsg.linear_acceleration.x = self.mag.x
            vizmsg.linear_acceleration.y = self.mag.y
            vizmsg.linear_acceleration.z = self.mag.z

        self.mag_pub.publish(self.msg_mag)
        self.magviz_pub.publish(vizmsg)

if __name__=='__main__':

    mag = MagCalibrated()
    while not rospy.is_shutdown():
        rospy.spin()

    parameters = {'magx_max':mag.magx_max,
                  'magx_min':mag.magx_min,
                  'magy_max':mag.magy_max,
                  'magy_min':mag.magy_min,
                  'magz_max':mag.magz_max,
                  'magz_min':mag.magz_min}

    sio.savemat('imu'+mag.num+'calibration.mat',parameters)
    rospy.sleep(2)
    print "Done saving data"
                  
        

