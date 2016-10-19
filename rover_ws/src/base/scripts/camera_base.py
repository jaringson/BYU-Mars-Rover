#!/usr/bin/env python	
import rospy, sys, time
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy



"""
This node is meant to receive compressed image data from all 5 cameras on the rover and receive joy commands to 
decide which compressed images to send to the control station. 

"""

class cam_hub:

	def __init__(self):


		# array of camera 0-1, If camera array at index 0 is high, then camera 0 should be publishing
		self.camera_array = [0,0]

		# flag for first time through loop
		self.first_time = True

		# publishers
		self.image0_pub = rospy.Publisher("hub_cam0/image_raw/compressed", CompressedImage, queue_size = 1000)
		self.image1_pub = rospy.Publisher("hub_cam1/image_raw/compressed", CompressedImage, queue_size = 1000)
		#subscribers
		self.cam0_sub = rospy.Subscriber("joy",Joy, self.joy_callback)
		self.cam0_sub = rospy.Subscriber("usb_cam0/image_raw/compressed",CompressedImage, self.image_callback)
		self.cam1_sub = rospy.Subscriber("usb_cam1/image_raw/compressed",CompressedImage, self.image_callback)
	

	"""
	joy_callback is called whenever the switch-camera button is pressed on the  xbox controller. 
	It must be able to cylce through all cameras without cutting out or increasing latency.

	"""
	def joy_callback(self, data):
		# button_pushed = data
		if self.first_time:
			i = 0
			print(" i = %d", i)
			self.first_time = False
		elif data.buttons[0]:
			rospy.loginfo(" button was pushed")
			i = i + 1
			print(" i = ", i)



		# 	image_pub.publish()	

		print("data = ", data.buttons[0])

	def image_callback(self, data):
		#rospy.loginfo(rospy.get_caller_id() + "hello world")
		# if button_pushed:

		self.image0_pub.publish(data)

		self.image1_pub.publish(data)	

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('cam_hub', anonymous=True)
	ic = cam_hub()
	rospy.loginfo("got here")		
	try:
		rospy.spin()

	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"


	
if __name__ == '__main__':

	main(sys.argv)
