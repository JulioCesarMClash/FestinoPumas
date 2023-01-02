#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('pos_definition')
import sys
import rospy
import ros_numpy
import numpy as np
import math
import tf

from geometry_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Point


class position_control:

  def move_distance(goal_dist, goal_angle, pub_goal_dist):
    msg_dist = Float32MultiArray()
    msg_dist.data = [goal_dist, goal_angle]
    print("Publishing distance")
    pub_goal_dist.publish(msg_dist)

  def __init__(self):
    self.piece_pos_sub = rospy.Subscriber("/redpiece_pos",Point,self.callback_piece_pos)

  def callback_piece_pos(self,data):
    piece_pos = [data.x, data.y,data.z]
    print(piece_pos)


def main(args):
  print("Position control - Aligning Gripper")
  global position
  pc = position_control()
  rospy.init_node('position_control', anonymous=True)
  pub_goal_dist = rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
  #distance = math.sqrt(waiving_point.x**2 + waiving_point.y**2) - 1.0
  #angle = math.atan2(waiving_point.y, waiving_point.x)
  #move_distance(distance, angle, pub_goal_dist)
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)