#!/usr/bin/env python	
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import joy


"""
This node is meant to receive compressed image data from all 5 cameras on the rover and receive joy commands to 
decide which compressed images to send to the control station. 

"""

def image_callback(data):
	rospy.loginfo(rospy.get_caller_id() + "hello world")


def cam_hub():

	rospy.init_node('cam_hub', anonymous=True)

	
	rospy.Subscriber("usb_cam0/image_raw/compressed",CompressedImage, image_callback)
	rospy.Subscriber("usb_cam1/image_raw/compressed",CompressedImage, image_callback)

	rospy.spin()
	
if __name__ == '__main__':

	cam_hub()
