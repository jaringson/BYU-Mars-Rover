#!/usr/bin/env python

import serial, rospy, struct
from rover_msgs.msg import Drive, RoverState, PSOC # Pololu, SciFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray, Int8
import re
import numpy as np

class GPS_redirect():

	def __init__(self):
	
		# initialize subscriber
		self.sub_drive = rospy.Subscriber('/fix_noframe', NavSatFix, self.gps_callback)       

        	# initialize publishers
		self.pub_psoc = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    # Callback
    
	def self.gps_callback(self, msg):

		fix = msg
		fix.header.frame_id = "gps"

		self.pub_psoc.publish(fix)


if __name__ == '__main__':
	rospy.init_node('psoc_node', anonymous=True)

	gps = GPS_redirect()

	while not rospy.is_shutdown():

		rospy.spin()

