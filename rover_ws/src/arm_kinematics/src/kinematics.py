import numpy as np, math
import rospy
import PyKDL as kdl
from urdf_praser_py.urdf import URDF


class ArmKinematics():
    def __init__(self):
        
        
    def create_chain(self):
        ch - kdl.Chain()
        cd.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame.DH(
