#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float32MultiArray
import lib_robotis as lr
import lib_dynamixel as ld
import math
from rover_msgs.msg import RoverState
from rover_msgs.srv import PositionReturn, PositionReturnResponse
from sensor_msgs.msg import JointState
import time

class DynPub():
    def __init__(self, wrist=True, gimbal=True):
        # Init node
        rospy.init_node('dynamixel_feedback_node',anonymous = True)
        hz = 100.0
        self.rate = rospy.Rate(hz)

        # Init dynamixel objects
        ids = []
        if wrist:
            ids.append(1)
            ids.append(2)
        if gimbal:
            ids.append(3)
            ids.append(4)

        self.dyn = ld.Dynamixel_Chain(dev='/dev/ttyUSB0',ids=ids)

        # Set wrist to multi-turn
        if wrist:
            self.dyn.write_address(1,6,[255,15]*2)
            self.dyn.write_address(2,6,[255,15]*2)

            self.wrist_command = [0,0]
            self.wrist_feedback = Float32MultiArray()
            self.wrist_feedback.data.append(0.0)
            self.wrist_feedback.data.append(0.0)

            self.pub_wrist  = rospy.Publisher('/wrist_feedback', Float32MultiArray, queue_size = 1)
            self.sub_wrist  = rospy.Subscriber('/joint_cmd', JointState, self.wristCallback)
            self.srv_wrist  = rospy.Service('wrist_position', PositionReturn, self.wrist_position_srv)

        if gimbal:
            self.gimbal_command = [0,0]
            self.gimbal_feedback = Float32MultiArray()
            self.gimbal_feedback.data.append(0.0)
            self.gimbal_feedback.data.append(0.0)

            self.pub_gimbal = rospy.Publisher('/gimbal_feedback', Float32MultiArray, queue_size = 1)
            self.sub_gimbal = rospy.Subscriber('/rover_state_cmd', RoverState, self.gimbalCallback)

        self.wrist_enabled = wrist 
        self.gimbal_enabled = gimbal
        self.ready = {'wrist': False, 'gimbal': False}
        
        
    def wristCallback(self,msg):
        theta5 = msg.position[4] #% (2*math.pi) #hinge
        theta6 = msg.position[5] #% (2*math.pi) #twist
        
        self.wrist_command[0] = (theta6+theta5) #dyn1
        self.wrist_command[1] = (theta5-theta6) #dyn2
        self.ready['wrist'] = True

    def wrist_position_srv(self, srv):
        D1 = self.wrist_feedback.data[0]
        D2 = self.wrist_feedback.data[1]
        wrist = [(D1+D2)/2, (D1-D2)/2]
        response = PositionReturnResponse(wrist, True, 'Wrist Feedback')
        # response.position = 1.1
        # response.success = True
        # response.tag = 'Wrist Feedback'
        return response

        
    def gimbalCallback(self, msg):
        self.gimbal_command[0] = msg.pan
        self.gimbal_command[1] = msg.tilt
        self.ready['gimbal'] = True

            
    def diffangle(self,angleTo,angleCur):
        # Returns the closest angle between two angles
        # negative is clockwise, positive is counter-clockwise
        # not used but left in since it's a useful function
        diff = angleTo - angleCur
        dist = atan2(sin(diff),cos(diff))
        return dist*180/pi

    def execute(self):
        while not rospy.is_shutdown():
            # Wrist
            if self.wrist_enabled:
                self.wrist_feedback.data[0] = self.dyn.read_angle(1)
                self.wrist_feedback.data[1] = self.dyn.read_angle(2)
                self.pub_wrist.publish(self.wrist_feedback)
                if self.ready['wrist']:
                    self.dyn.move_angle(1,self.wrist_command[0], blocking = False)
                    self.dyn.move_angle(2,self.wrist_command[1], blocking = False)

            # Gimbal
            if self.gimbal_enabled:
                self.gimbal_feedback.data[0] = self.dyn.read_angle(4)
                self.gimbal_feedback.data[1] = self.dyn.read_angle(3)
                self.pub_gimbal.publish(self.gimbal_feedback)
                if self.ready['gimbal']:
                    self.dyn.move_angle(4,self.gimbal_command[0], blocking = False)
                    self.dyn.move_angle(3,self.gimbal_command[1], blocking = False)

            self.rate.sleep()
    

if __name__ == "__main__":
    dynpub = DynPub(True,False)
    dynpub.execute()
    
