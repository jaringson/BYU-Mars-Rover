#!/usr/bin/env python

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import random

robot = URDF.from_xml_file('/home/halrover/BYU-Mars-Rover/rover_ws/src/hal_description/urdf/hal.urdf')
tree = kdl_tree_from_urdf_model(robot)

print tree.getNrOfSegments()

base_link = robot.get_root()
end_link = robot.link_map.keys()[random.randint(0, len(robot.link_map)-1)]

chain = tree.getChain(base_link, end_link)
print chain.getNrOfJoints()

print "Root link: %s; Random end link: %s" % (base_link, end_link)
kdl_kin = KDLKinematics(robot, base_link, end_link)
q = kdl_kin.random_joint_angles()
print "Random angles:", q
pose = kdl_kin.forward(q)
print "FK:", pose
q_new = kdl_kin.inverse(pose)
print "IK (not necessarily the same):", q_new
if q_new is not None:
    pose_new = kdl_kin.forward(q_new)
    print "FK on IK:", pose_new
    print "Error:", np.linalg.norm(pose_new * pose**-1 - np.mat(np.eye(4)))
else:
    print "IK failure"
J = kdl_kin.jacobian(q)
print "Jacobian:", J
M = kdl_kin.inertia(q)
print "Inertia matrix:", M
if False:
    M_cart = kdl_kin.cart_inertia(q)
    print "Cartesian inertia matrix:", M_cart
