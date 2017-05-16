# #!/usr/bin/env python

from PyKDE4.marble import *
from PyQt4.QtCore import *
import rospy
from python_qt_binding import loadUi
from PyQt4.Qt import *
from PyQt4 import QtGui
from rover_msgs.msg import NavState, ArmState, RoverState
from msg import FloatList
from math import ceil, floor, sqrt, sin, asin, cos, acos, radians, degrees, pi

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = type("")

import os
import map_info_parser
from std_msgs.msg import String
from msg import FW_State, GPS
from Signals import WP_Handler
from msg import FW_Waypoint


import sys, math, rospy, random
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import pyqtSlot, pyqtSignal, Qt, QPointF,QRectF, QPoint
from PyQt4.QtGui import QColor, QBrush, QPen, QFont, QPolygon


PWD = os.path.dirname(os.path.abspath(__file__))

class TiltReadout(QWidget):
    def __init__(self, _marble_map, uifname = 'ahnew.ui'):
        super(TiltReadout, self).__init__()
#        self._marble_map = _marble_map
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)
        self.wsms = rospy.Subscriber("/rover_state_cmd", RoverState, self.callbackrover)
        self.asms = rospy.Subscriber("/arm_state_cmd", ArmState, self.callbackarm)
        self.ss = rospy.Subscriber("/estimate", NavState, self.callbacknav)

    def callbacknav(self, state):
        self.pe = state.position[1]
        self.pn = state.position[0]
        self.blat = state.base_latitude
        self.blon = state.base_longitude
        self.distfrombase = (self.pe**2 + self.pn**2)**(0.5)
        self.roll = state.phi
        self.pitch = state.theta
        self.yaw = state.psi
        self.label_2.setText('%d degrees' % (self.roll*180/3.14159265358979))
        self.label_4.setText('%d degrees' % (self.pitch*180/3.14159265358979))
        self.label_6.setText('%d degrees' % (self.yaw*180/3.14159265358979))
        self.label_8.setText('%d meters' % (self.distfrombase))

    def callbackarm(self, astate):
        self.armspeedmode = astate.mode
        print self.armspeedmode
        self.label_12.setText(self.armspeedmode)

    def callbackrover(self, wstate):
        self.wheelsspeedmode = wstate.mode
        print self.wheelsspeedmode
        self.label_10.setText(self.wheelsspeedmode)




#    def python_code(self):
#        rospy.init_node("python_code",anonymous=True)
#        rospy.Subscriber("/rover_state_cmd", RoverState, self.callbackrover)
#        rospy.Subscriber("/arm_state_cmd", ArmState, self.callbackarm)
#        rospy.Subscriber("/estimate", NavState, self.callbacknav)





class wspeedmodeSubscriber():
    def __init__(self):
        self.wheelsspeedmode = 'none'
        rospy.Subscriber("/rover_state_cmd", RoverState, self.callback)
    def callback(self, rstate):
        self.wheelsspeedmode = rstate.mode
        print self.wheelsspeedmode


class aspeedmodeSubscriber():
    def __init__(self):
        self.armspeedmode = 'none'
        rospy.Subscriber("/arm_state_cmd", ArmState, self.callback)
    def callback(self, astate):
        self.armspeedmode = astate.mode
        print self.armspeedmode


class StateSubscriber(): # For rendering rotated plane onto marble widget
    def __init__(self):
        self.pe = 0.0
        self.pn = 0.0
        self.psi = 0.0
        self.blat = 38.4065
        self.blon = -110.7919

        rospy.Subscriber("/estimate", NavState, self.callback)

    def callback(self, state):
        self.pe = state.position[1]
        self.pn = state.position[0]
        self.blat = state.base_latitude
        self.blon = state.base_longitude
        self.distfrombase = (self.pe**2 + self.pn**2)**(0.5)
        self.roll = state.phi
        self.pitch = state.theta
        self.yaw = state.psi
        self.label_2.setText('%d degrees' % (self.roll*180/3.14159265358979))
        self.label_4.setText('%d degrees' % (self.pitch*180/3.14159265358979))
        self.label_6.setText('%d degrees' % (self.yaw*180/3.14159265358979))
        self.label_8.setText('%d meters' % (self.distfrombase))
        print "hi"



#if __name__ == '__main__':
#    print 'Running dashboard'
#    python_code()
