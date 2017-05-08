import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('rover_transforms')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not ros.is_shutdown():
        # LiDar to Base Frame
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         "lidar_base",
                         "rover_base")
        rate.sleep()
