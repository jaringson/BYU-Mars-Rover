import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rover_msgs.msg import Drive

class WheelControl:
    def __init__(self):
        # Set Parameters
        self.L = 0.6096  # Distance between wheels (2 ft)
        self.R = 0.1524  # Radius of wheels
        self.max_vel = 1  # Maximum turn rate, corresponds to 100 in /rover_msg/Drive. Radians per second

        # init ROS node
        rospy.init_node('navigation')

        # set rate
        hz = 10.0  # 60.0
        self.rate = rospy.Rate(hz)

        # Set up Subscriber
        self.sub_cmd = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Set up Publisher
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Spin the Node
        rospy.spin()

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        vr = (2*v + w*self.L)/(2*self.R) # Turn rate of right wheel
        vl = (2*v - w*self.L)/(2*self.R) # Turn rate of left wheel

        rw = self.sat(vr/self.max_vel, 100)
        lw = self.sat(vl/self.max_vel, 100)

        # Create Drive Message
        cmd = Drive()
        cmd.rw = rw
        cmd.lw = lw

    @staticmethod
    def sat(x, lim):
        if x > lim:
            x = lim
        elif x < lim:
            x = -lim
        return x

if __name__ == '__main__':
    W = WheelControl()