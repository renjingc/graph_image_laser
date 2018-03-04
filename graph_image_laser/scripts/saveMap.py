#!/usr/bin/env python

import sys
import rospy
from graphImageLaser.srv import *


if __name__ == "__main__":
    rospy.wait_for_service('save_map')
    try:
        save_map_ser = rospy.ServiceProxy('/graph_image_laser/save_map', SaveMap)
        save_map_ser("/home/ren/knowmeking")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
