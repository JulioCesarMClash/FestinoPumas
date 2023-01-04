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
import tf_conversions
import tf2_ros

from geometry_msgs.msg import *
from std_msgs.msg import *

## Publica la TF, verifica que se mueva
## Control PID


#class position_control:

#  def __init__(self):
#    self.piece_pos_sub = rospy.Subscriber("/redpiece_pos",PointStamped,self.callback_piece_pos)
#    self.piece_position = [0,0,0]

def callback_piece_pos(data):
  global piece_position
  piece_position = [data.point.x, data.point.y,data.point.z]
  print(piece_position)
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(1.0)
  jelp = br.sendTransform((data.point.x, data.point.y, data.point.z),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),
                     "piece_rgbd_sensor_link", "camera_link")
  print(jelp)


def main(args):
  rospy.init_node('position_control', anonymous=True)
  print("Position control - Aligning Gripper")
  piece_pos_sub = rospy.Subscriber("/redpiece_pos",PointStamped,callback_piece_pos)
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)