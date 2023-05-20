#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from img_proc.srv import *

def add_two_ints_client(x, y, ask_zone):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y, ask_zone)
        return resp1.sum, resp1.det_flag
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        ask_zone = True
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    sum_res, flag = add_two_ints_client(x, y,ask_zone)
    print("%s + %s = %s"%(x, y, sum_res), flag)