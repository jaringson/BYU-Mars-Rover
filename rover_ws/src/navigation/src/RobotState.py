from geometry_msgs.msg import Pose, Twist
import tf.transformations as tr

class RobotState:
    def __init__(self):
        self.pose = Pose()
        self.twist = Twist()

        self.goal = [0.0, 0.0]

    def GetPose(self):
        x, y = self.pose.position.x, self.pose.position.y
        phi, theta, psi = tr.euler_from_quaternion(self.pose.orientation, 'rzyx')
        return [x, y, theta]


