import numpy as np
import tf.transformations as tr
from PID import PID


class Controller:
    def __init__(self, params):
        self.pid = PID(params)
        self.name = "default"

    def get_heading(self, robot):
        x, y, theta = robot.GetPose()
        return theta

    def get_outputs(self, robot):
        u, v = 0, 0
        return [u, v]
