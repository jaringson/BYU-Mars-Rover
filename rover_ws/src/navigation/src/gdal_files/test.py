#!/usr/bin/env python

from grid_map_msgs.msg import GridMap
import rospy
import scipy.io as io

data_set = io.loadmat('topo_data_geodem.mat')


def test_publisher():
	pub = rospy.Publisher('/test_grid_map', GridMap, queue_size=10)
	rospy.init_node('grid_map_test_publisher')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(map)
		rate.sleep()

if __name__ == '__main__':
	map = GridMap()
	map.info.header.frame_id = 'world'
	map.info.resolution = data_set['numGrids']
	map.info.length_x = data_set['x-max']
	map.info.length_y = data_set['y-max']
	map.data = data_set['z']
	try:
		test_publisher()
	except rospy.ROSInterruptException:
		pass



'''
rospy.Subscriber('/grid_map_simple_demo/grid_map', GridMap, callback)

rospy.spin()


def callback(msg):
	pub.publish(msg)
'''
