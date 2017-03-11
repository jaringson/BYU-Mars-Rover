#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float32MultiArray
import lib_robotis as lr
import lib_dynamixel as ld
import math
from sensor_msgs.msg import JointState
import time

class DynPub():
    def __init__(self):
        self.zeroangs = [2.80633601668245, -1.1047358781854217]
        self.angles_current = Float32MultiArray()
        self.angles_current.data.append(0.0)
        self.angles_current.data.append(0.0)
        self.angle_command = Float32MultiArray()
        self.angle_command.data.append(0.0)
        self.angle_command.data.append(0.0)
        self.pub = rospy.Publisher('/dynamixel_feedback',Float32MultiArray,queue_size = 1)
        
        self.sub = rospy.Subscriber('/joint_cmd',JointState,self.dynCallback)
        self.resets = 0
        
    def dynCallback(self,msg):
        theta5 = msg.position[4] #% (2*math.pi) #hinge
        theta6 = msg.position[5] #% (2*math.pi) #twist
        
        self.angle_command.data[0] = (theta6+theta5) #dyn1
        self.angle_command.data[1] = (theta5-theta6) #dyn2
        
        #print self.c_angles

    def reset_limit(self, dyn, id):
        cur_enc = dyn.read_encoder(id)
        #print cur_enc
        if abs(cur_enc) > 4095:
            dyn.reset_encoder(id)
            time.sleep(0.1)
            self.resets += 1
            
    def diffangle(self,angleTo,angleCur):
        # negative is clockwise, positive is counter-clockwise
        #angleTo = radians(10)
        #angleCur = radians(340)
        diff = angleTo - angleCur
        dist = atan2(sin(diff),cos(diff))
        return dist*180/pi

    def execute(self):
        while not rospy.is_shutdown():
            dynpub.angles_current.data[0] = dyn.read_angle(1)
            dynpub.angles_current.data[1] = dyn.read_angle(2)
            dynpub.pub.publish(dynpub.angles_current)
            print dynpub.angles_current
            #print dyn.read_encoder(1), dyn.read_encoder(2)
            dyn.move_angle(1,dynpub.angle_command.data[0], blocking = False)
            dyn.move_angle(2,dynpub.angle_command.data[1], blocking = False)

            rate.sleep()
    

if __name__ == "__main__":
    rospy.init_node('dynamixel_feedback_node',anonymous = True)
    hz = 100.0
    rate = rospy.Rate(hz)
    dynpub = DynPub()
    
    dyn = ld.Dynamixel_Chain(dev='/dev/ttyUSB1',ids=[1,2])
    dyn.write_address(1,6,[255,15]*2)
    dyn.write_address(2,6,[255,15]*2)
    
    print "Successful"
    dynpub.execute()
    
