#!/usr/bin/env python  
import roslib
roslib.load_manifest('movement_functions')
import rospy
import math
import tf
import geometry_msgs.msg
import sys

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    zone = '/'+sys.argv[1]
    print(zone)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(zone, '/map', rospy.Time(0))
            print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
