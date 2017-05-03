#!/usr/bin/python

import json
import roslib
import rospy
import sys
import numpy as np
from global_wp_pub.msg import FloatList
from pprint import pprint

class json_parse:
    def __init__(self):
        self.wp_pub = rospy.Publisher("/global_path",FloatList,queue_size=1,latch=True)

        self.import_json()

    def import_json(self):

        data = json.load(open('/home/peter/git/BYU-Mars-Rover/onboard_ws/src/global_wp_pub/src/costpath2.json','r'))

        coords = data['features'][1]['geometry']['coordinates']

        pprint(coords)
        print(type(coords[0][0]))

        msg = FloatList()

        for l in coords:
            msg.data.append(l[0])
            msg.data.append(l[1])

        self.wp_pub.publish(msg)
        print('published')

def main(args):

    rospy.init_node('json_pub', anonymous=False)
    json_thing = json_parse()
    r = rospy.Rate(15)
    try:
        rospy.spin()
        r.sleep()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':

    main(sys.argv)
