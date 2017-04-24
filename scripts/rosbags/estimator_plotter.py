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
import sys

class rover_plot():

    def __init__(self):

        rospy.Subscriber("gps", Float32, self.gps_callback)
        #QtGui.QApplication.setGraphicsSystem('raster')
        app = QtGui.QApplication([])
        #mw = QtGui.QMainWindow()
        #mw.resize(800,800)

        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000,600)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        self.p1 = self.win.addPlot(title="Basic array plotting", x=range(0,100), y=np.random.normal(size=100),pen=None, symbol='o')

        self.p6 = self.win.addPlot(title="Updating plot")
        self.curve = self.p6.plot(pen=None, symbol='o')
        self.p6.addLine(x=0)
        self.p6.addLine(y=0)
        self.p6.setRange(xRange=[-100,100],yRange=[-100,100])
        # self.data = np.random.normal(size=(1000))
        self.datax = []
        self.datay = []
        self.ptr = 0
        
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(50)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def update(self):
            self.datax.append(self.ptr)
            self.datay.append(self.ptr)
            self.curve.setData(self.datax, self.datay)
            if self.ptr == 0:
                self.p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
            self.ptr += 1

    def gps_callback(self, msg):
        pass

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    # import sys

    # Initialize Node
    rospy.init_node('estimator_plots')

    plot = rover_plot()

    # Loop
    while not rospy.is_shutdown():
        rospy.spin()