from geometry_msgs.msg import Pose, Twist
from rover_msgs.msg import NavState
import tf.transformations as tr
import math

class RobotState:
    def __init__(self, goal_distance=0.2, def_v=0.5):
        self.pose = Pose()
        self.twist = Twist()
        self.pose.orientation.w = 1
        self.default_v = def_v
        self.state = NavState()

        self.goal = [0.0, 0.0]

        self.goal_distance = goal_distance
        self.use_NavState = True

    def set_goal(self, goal):
        self.goal = goal

    def get_pose(self):
        if (self.use_NavState):
            x, y = self.state.position[0], self.state.position[1]
            theta = self.state.psi
        else:
            x, y = self.pose.position.x, self.pose.position.y
            theta = self.get_heading()
        return [x, y, theta]

    def get_heading(self):
        quaternion = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w)
        psi, theta, phi = tr.euler_from_quaternion(quaternion, 'rzyx')
        psi = self.state.psi
        return psi

    def dist_to_goal(self):
        x, y , theta = self.get_pose()
        return math.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2)

    def at_goal(self):
        return self.dist_to_goal() < self.goal_distance

