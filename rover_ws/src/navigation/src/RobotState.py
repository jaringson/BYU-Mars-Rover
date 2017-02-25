from geometry_msgs.msg import Pose, Twist
import tf.transformations as tr
import math

class RobotState:
    def __init__(self, goal_distance=0.2):
        self.pose = Pose()
        self.twist = Twist()
        self.pose.orientation.w = 1

        self.goal = [0.0, 0.0]

        self.goal_distance = goal_distance

    def set_goal(self, goal):
        self.goal = goal

    def get_pose(self):
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
        return psi

    def dist_to_goal(self):
        x, y , theta = self.get_pose()
        return math.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2)

    def at_goal(self):
        return self.dist_to_goal() < self.goal_distance

