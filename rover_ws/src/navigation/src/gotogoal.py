import numpy
import math
from geometry_msgs.msg import Pose, Twist
import tf.transformations as tr

class GoToGoal:
    def __init__(self, params):
        self.vector_to_goal = [0,0]

    def get_heading(self, robotState):
        """Return the heading as the angle to goal in the robot's reference frame"""
        
        # Extract goal
        x_goal, y_goal = robotState.goal[0], robotState.goal[1]
        
        # Robot pose
        x_cur, y_cur = robotState.pose.position.x, robotState.pose.position.y
        phi, theta, psi = tr.euler_from_quaternion(robotState.pose.orientation,'rzyx')
        
        # Vector to Goal
        self.vector_to_goal = [x_goal-x_cur,y_goal-ycur]
        angle = self.get(angle)-theta
        return atan2(cos(angle),sin(angle))
        
    def get_angle(self):
        return atan2(self.vector_to_goal[1],self.vector_to_goal[0])

    def get_outputs(self):
        
