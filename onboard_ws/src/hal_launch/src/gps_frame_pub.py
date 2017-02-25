#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
import rospy

class GPS_redirect():

	def __init__(self):

		# initialize subscriber
		self.sub_drive = rospy.Subscriber('/fix_noframe', NavSatFix, self.gps_callback)       

		# initialize publishers
		self.pub_psoc = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    # Callback

	def gps_callback(self, msg):

		fix = msg
		fix.header.frame_id = "gps"

		self.pub_psoc.publish(fix)


if __name__ == '__main__':
	rospy.init_node('gps_repub')

	gps = GPS_redirect()

	while not rospy.is_shutdown():

		rospy.spin()

