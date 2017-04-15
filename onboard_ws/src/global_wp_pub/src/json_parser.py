#!/usr/bin/python

import json
import roslib
import rospy
import sys
import numpy as np
# sys.path.append('..')
from std_msgs.msg import Float64MultiArray
from global_wp_pub.msg import FloatList
from pprint import pprint

class json_parse:
    def __init__(self):
        self.wp_pub = rospy.Publisher("/global_path",FloatList,queue_size=1,latch=True)

        self.import_json()

    def import_json(self):

        data = json.load(open('/home/peter/git/BYU-Mars-Rover/onboard_ws/src/global_wp_pub/src/costpath2.json','r'))

        coords = data['features'][1]['geometry']['coordinates']

        # json.dumps(data)
        pprint(coords)
        # print(coords[0][0])
        print(type(coords[0][0]))

        msg = FloatList()

        # msg.data = [coords[0][0],coords[0][1]]

        for l in coords:
            msg.data.append(l[0])
            msg.data.append(l[1])

        # superdata = np.array([[110,80],[100,80.0]])

        # msg.data = superdata
        # msg.layout.dim[0].label = "length"
        # msg.layout.dim[0].size = len(coords)
        # msg.layout.dim[0].stride = len(coords)*2
        # msg.layout.dim[1].label = "two"
        # msg.layout.dim[1].size = 2
        # msg.layout.dim[1].stride = 2

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
    #cv2.destroyAllWindows()


if __name__ == '__main__':

    main(sys.argv)
