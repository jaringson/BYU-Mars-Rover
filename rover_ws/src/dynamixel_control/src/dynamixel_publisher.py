#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float32MultiArray
import lib_robotis as lr
import math
from sensor_msgs.msg import JointState

class DynPub():
    def __init__(self):
        self.r_angles = Float32MultiArray()
        self.c_angles = Float32MultiArray()
        self.c_angles.data.append(0.0)
        self.c_angles.data.append(0.0)
        self.pub = rospy.Publisher('/dynamixel_feedback',Float32MultiArray,queue_size = 1)

        self.sub = rospy.Subscriber('/joint_cmd',JointState,self.dynCallback)

    def dynCallback(self,msg):
        theta5 = msg.position[4] #hinge
        theta6 = msg.position[5] #twist
        
        self.c_angles.data[0] = theta6+theta5 #dyn1
        self.c_angles.data[1] = theta5-theta6 #dyn2
        
        #print self.c_angles

if __name__ == "__main__":
    rospy.init_node('dynamixel_feedback_node',anonymous = True)
    hz = 60.0
    rate = rospy.Rate(hz)
    dynpub = DynPub()

    dynpub.r_angles.data.append(0.0)
    dynpub.r_angles.data.append(0.0)
    
    dyn = lr.USB2Dynamixel_Device('/dev/ttyUSB0',57600)
    #dyn1 = old code's twist
    #dyn2 = old code's flop
    
    dyn2 = lr.Robotis_Servo(dyn, 1, series = 'MX')
    #dyn2 = dyn2.write_address(0x0E, [255,3])
    dyn1 = lr.Robotis_Servo(dyn, 2, series = 'MX')
    #dyn1 = dyn1.write_address(0x0E, [255,3])
    dyn1.multi_turn()
    dyn2.multi_turn()
    
    print "Successful"
    while not rospy.is_shutdown():
        dynpub.r_angles.data[0] = dyn2.read_angle()
        dynpub.r_angles.data[1] = dyn1.read_angle()
        dynpub.pub.publish(dynpub.r_angles)
        dyn1.move_angle(dynpub.c_angles.data[1], blocking = False)
        dyn2.move_angle(dynpub.c_angles.data[0], blocking = False)
        

        rate.sleep()
