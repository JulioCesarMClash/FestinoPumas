#!/usr/bin/env python

import roslaunch
import rospy
import matplotlib
import os
import sys
import tf2_ros
import tf_conversions
import tf
import math
import numpy as np
from geometry_msgs.msg import *

path = '/home/pumas/FestinoPumas/PC_user/src/'

tfBuffer = tf2_ros.Buffer()

"""def final_robot_pos():
	listener = tf.TransformListener()
	now = rospy.Time.now()
	listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
	(trans,rot) = listener.lookupTransform("/map", "/base_link", now)
	print("Robot a map", trans)"""


rospy.init_node('step_navigation', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

ros_nav_launch 		= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/config_files/launch/explore_n_map.launch"])
doc_nav_launch 		= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/config_files/launch/late_navigation.launch"])
log_zones_launch 	= roslaunch.parent.ROSLaunchParent(uuid, [path + "Navigation/Pos_control/movement_functions/launch/logisticsZones.launch"])

position_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=10)

listener = tf.TransformListener()

robot_init_pose = PoseWithCovarianceStamped()
robot_init_pose.header.seq = 1
robot_init_pose.header.stamp = rospy.Time.now()
robot_init_pose.header.frame_id = "map"

ros_nav_launch.start()
rospy.loginfo("ROS Exploration started")
rospy.loginfo("Robot-Server communication started")

now = rospy.get_rostime()
rospy.loginfo("Mapping started at %i", now.secs)

rospy.sleep(60)

now = rospy.Time.now()
listener.waitForTransform("/odom", "/map", now, rospy.Duration(4.0))
(first_trans,first_rot) = listener.lookupTransform("/odom", "/map", now)
print("Robot a map - FIRST POS", first_trans, "\t", first_rot, "at %i", now.secs)

os.system("rosrun map_server map_saver -f " + path + "Navigation/config_files/maps/dirty_latemap")
os.system("rosrun map_server map_saver -f " + path + "Navigation/config_files/prohibition_maps/dirty_latemap_pro")
rospy.loginfo("Map saved")

now = rospy.get_rostime()
rospy.loginfo("Mapping killed at %i", now.secs)
ros_nav_launch.shutdown()

now = rospy.get_rostime()
rospy.loginfo("Doc Nav started at %i", now.secs)
doc_nav_launch.start()
#log_zones_launch.start()
rospy.sleep(10)

now = rospy.Time.now()
listener.waitForTransform("/odom", "/map", now, rospy.Duration(4.0))
(first_trans,first_rot) = listener.lookupTransform("/odom", "/map", now)
print("\n Robot a map - SEC POS", first_trans, "\t", first_rot, "at %i", now.secs)

print("WRITING ORIGIN IN PREV-MAP")
robot_init_pose.pose.pose.position.x = first_trans[0]
robot_init_pose.pose.pose.position.y = first_trans[1]
robot_init_pose.pose.pose.position.z = first_trans[2]
robot_init_pose.pose.pose.orientation.x = first_rot[0]
robot_init_pose.pose.pose.orientation.y = first_rot[1]
robot_init_pose.pose.pose.orientation.z = first_rot[2]
robot_init_pose.pose.pose.orientation.w = first_rot[3]
robot_init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
position_pub.publish(robot_init_pose)

rospy.sleep(3)

now = rospy.Time.now()
listener.waitForTransform("/odom", "/map", now, rospy.Duration(4.0))
(first_trans,first_rot) = listener.lookupTransform("/odom", "/map", now)
print("Robot a map - LAST POS", first_trans, "\t", first_rot, "at %i", now.secs)

try:
    rospy.spin()
except KeyboardInterrupt:
	launch.shutdown()
	print("Shutting down")
