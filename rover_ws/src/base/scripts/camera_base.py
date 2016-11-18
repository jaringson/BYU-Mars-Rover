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

		self.joy = Joy()

		# array of camera 0-1, If camera array at index 0 is high, then camera 0 should be publishing
		self.camera_array = [0,0]
		self.counter = 0
		self.max_counter = 1
		# flag for first time through loop
		# self.first_time = True
		self.camera_on = False
		# publishers
		self.pub_image0 = rospy.Publisher("hub_cam0/image_raw/compressed", CompressedImage, queue_size = 10)
		self.pub_image1= rospy.Publisher("hub_cam1/image_raw/compressed", CompressedImage, queue_size = 10)
		#subscribers
		self.sub_joy = rospy.Subscriber("joy",Joy, self.joy_callback)
		self.sub_cam0 = rospy.Subscriber("usb_cam0/image_raw/compressed",CompressedImage, self.image_callback)
		self.sub_cam1 = rospy.Subscriber("usb_cam1/image_raw/compressed",CompressedImage, self.image_callback)
	

	"""
	joy_callback is called whenever the switch-camera button is pressed on the  xbox controller. 
	It must be able to cylce through all cameras without cutting out or increasing latency.

	"""
	def joy_callback(self, data):
		# button_pushed = data
		self.joy=data
		if self.joy.buttons[0] == 1:
			if self.camera_on==False: 
				self.camera_on=True
			else:
				self.camera_on=False
		

	def image_callback(self, data):
		#rospy.loginfo(rospy.get_caller_id() + "hello world")
		# if button_pushed:
		cycle_button = self.joy.buttons[0]
		print("cycle button", cycle_button)
		if self.camera_on:
			# self.counter = self.counter + 1
			print("got here")           
			# if self.counter == 1:
			self.pub_image0.publish(data)

		else:

			self.pub_image1.publish(data)
			# self.counter = 0
		time.sleep(.25)

		

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('cam_hub', anonymous=True)
	ic = cam_hub()		
	try:
		rospy.spin()

	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"


	
if __name__ == '__main__':

	main(sys.argv)
