#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from rover_msgs.msg import ArmState
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
from math import *

def RPYtoQuaternion(msg,roll,pitch,yaw):
    t0 = cos(yaw*0.5)
    t1 = sin(yaw*0.5)
    t2 = cos(roll*0.5)
    t3 = sin(roll*0.5)
    t4 = cos(pitch*0.5)
    t5 = sin(pitch*0.5)
    msg.orientation.w = t0 * t2 * t4 + t1 * t3 * t5
    msg.orientation.x = t0 * t3 * t4 - t1 * t2 * t5
    msg.orientation.y = t0 * t2 * t5 + t1 * t3 * t4
    msg.orientation.z = t1 * t2 * t4 - t0 * t3 * t5
    return msg

def publisher_fcn(pub1,pub2):

    # JointState Example msg
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.effort = []
    pub1.publish(msg)

    # Pose Example msg
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    roll = (pi)/6.0
    pitch = 0.0
    yaw = 0.0
    pose = RPYtoQuaternion(pose,roll,pitch,yaw)
    pub2.publish(pose)

if __name__ == '__main__':

    # init ROS Node
    rospy.init_node('talker', anonymous=True)

    # set rate 
    hz = 10.0
    rate = rospy.Rate(hz)

    # choose ROS Topic to pub to
    ros_topic = '/joint_ik'
    pub1 = rospy.Publisher(ros_topic, JointState, queue_size=10)
    pub2 = rospy.Publisher('/pose_ik', Pose, queue_size=10)

    # loop
    while not rospy.is_shutdown():
        # do this
        publisher_fcn(pub1,pub2)
        
        rate.sleep()