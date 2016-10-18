#!/usr/bin/env python

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import random

# import robot from urdf file
robot = URDF.from_xml_file('/home/halrover/BYU-Mars-Rover/rover_ws/src/hal_description/urdf/rrbot.urdf')
tree = kdl_tree_from_urdf_model(robot)
print ''
print tree.getNrOfSegments()

# set base link and a random end link of robot
base_link = robot.get_root()
end_link = robot.link_map.keys()[random.randint(0, len(robot.link_map)-1)]

# make chain from base link to end link
chain = tree.getChain(base_link, end_link)
print ''
print chain.getNrOfJoints()
print "Root link: %s; Random end link: %s" % (base_link, end_link)

# random joint angles of robot
kdl_kin = KDLKinematics(robot, base_link, end_link)
#temp = (3.14)/2.0
q = [0.0, 0.0]
#q = kdl_kin.random_joint_angles()
print ''
print "Random angles:", q

# FK on random angles
pose = kdl_kin.forward(q)
print ''
print "FK:", pose

# IK on new pose from FK
q_new = kdl_kin.inverse(pose)
print ''
print "IK (not necessarily the same):", q_new

# calc error on IK
if q_new is not None:
    pose_new = kdl_kin.forward(q_new)
    print ''
    print "FK on IK:", pose_new
    print "Error:", np.linalg.norm(pose_new * pose**-1 - np.mat(np.eye(4)))
else:
    print ''
    print "IK failure"

# Jacobian of at random angles q
J = kdl_kin.jacobian(q)
print ''
print "Jacobian:", J

# Inertia Matrix
M = kdl_kin.inertia(q)
print ''
print "Inertia matrix:", M

if False:
    M_cart = kdl_kin.cart_inertia(q)
    print ''
    print "Cartesian inertia matrix:", M_cart
