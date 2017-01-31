import numpy as np
import tf.transformations as tr
from PID import PID

class Controller:
    def __init__(self):
        self.pid = PID()
        ki, kd, kp = 3, 4, 0
        self.pid.SetGains(ki, kd, kp)

    def GetHeading(self,robotState):
        x, y, theta = robotState.GetPose()
        return theta

    def GetOutputs(self):

        return [u,v]