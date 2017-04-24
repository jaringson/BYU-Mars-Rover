#!/usr/bin/env python
"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

# import initExample ## Add path to library (just for examples; you do not need this)


from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu
from rover_msgs.msg import NavState
import sys
from math import *
import tf

class rover_plot():

    def __init__(self):
        
        # QT GUI Init
        #QtGui.QApplication.setGraphicsSystem('raster')
        app = QtGui.QApplication([])
        #mw = QtGui.QMainWindow()
        #mw.resize(800,800)

        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000,600)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        ############
        ## Plot 1 ##
        ############
        self.p1 = self.win.addPlot(title="Basic array plotting")
        self.p1.setRange(xRange=[-1,1],yRange=[-1,1])
        self.p1.addLine(x=0)
        self.p1.addLine(y=0)
        self.head = self.p1.plot(pen='r')

        ############
        ## Plot 2 ##
        ############
        self.p6 = self.win.addPlot(title="Updating plot")

        # 3 Parts (gps, truth, and estimate)
        self.gps = self.p6.plot(pen=[1,2], symbol='o')
        self.estimate = self.p6.plot(pen=[2,2], symbol='s')
        self.truth = self.p6.plot()

        # Estimator base GPS stuff
        self.base_lat = 40.247651667
        self.base_long = -111.64733
        self.EARTH_RADIUS = 6378145.0

        # Truth GPS coordinates
        self.line_lat = [40.247358, 40.247362] 
        self.line_long = [-111.647550, -111.647195] 

        line_n = [0, 0]
        line_e = [0, 0]

        # Compute local truth coordinates
        for i in range(0,len(self.line_lat)):
            line_n[i] = self.EARTH_RADIUS*(self.line_lat[i]-self.base_lat)*np.pi/180.0
            line_e[i] = self.EARTH_RADIUS*cos(self.base_lat*np.pi/180.0)*(self.line_long[i]-self.base_long)*np.pi/180.0

        # Draw Truth line Red
        self.truth.setData(line_e,line_n, pen='r')
        # Draw x,y axes
        self.p6.addLine(x=0)
        self.p6.addLine(y=0)
        # Set plot range
        self.p6.setRange(xRange=[-30,30],yRange=[-40,-20])

        # Init GPS data Array
        self.data_n = []
        self.data_e = []

        # Init heading Stuff
        self.psi = 0.0
        self.psi_deg = 0.0  

        # Init estimate stuff
        self.estimate_n = []
        self.estimate_e = []      

        # ROS Subscribers
        rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("new_estimate", NavState, self.estimate_callback)

        print 'init'
        # Time to update Plots
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(50)

        # IDK what this is haha
        # if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

        

    def update(self):
            self.head.setData([0, sin(self.psi)], [0, cos(self.psi)])
            self.gps.setData(self.data_e, self.data_n)
            self.estimate.setData(self.estimate_e, self.estimate_n)
            
    def gps_callback(self, msg):
        print 'gps'
        gps_n = self.EARTH_RADIUS*(msg.latitude-self.base_lat)*np.pi/180.0
        gps_e = self.EARTH_RADIUS*cos(self.base_lat*np.pi/180.0)*(msg.longitude-self.base_long)*np.pi/180.0
        # self.gps_h = msg.altitude - self.estimate.base_altitude
        # print msg.latitude
        self.data_n.append(gps_n)
        self.data_e.append(gps_e)

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        # self.estimate.phi = euler[0];
        # self.estimate.theta = -euler[1];
        self.psi = -euler[2] - (11.0*np.pi/180.0)
        self.psi_deg = self.psi * 180.0/np.pi

    def estimate_callback(self, msg):
        self.estimate_n.append(msg.position[0])
        self.estimate_e.append(msg.position[1])

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    # import sys

    # Initialize Node
    rospy.init_node('estimator_plots')

    plot = rover_plot()

    # Loop
    while not rospy.is_shutdown():
        rospy.spin()