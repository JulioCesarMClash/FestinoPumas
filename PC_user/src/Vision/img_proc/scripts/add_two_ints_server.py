#!/usr/bin/env python

from __future__ import print_function

from img_proc.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    det_flag = False
    if req.ask_zone:
        print('Recibi solicitud')
        det_flag = True
    else:
        print('No Recibi')
        det_flag = False
    return AddTwoIntsResponse(req.a + req.b, det_flag)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()