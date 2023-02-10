#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('img_proc')
import sys
import rospy
import ros_numpy
import numpy as np
import math
import tf
import tf_conversions
import tf2_ros
import cv2

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

def callback_image(data):
  global depth_img, arr, img_bgr, Red_mask
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
  parameters = cv2.aruco.DetectorParameters_create()
  markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=parameters)
  color = (255, 0, 0)
  thickness = 2
  if(markerIds.shape[0] > 1):
    for i in range (markerIds.shape[0]):
      esq = markerCorners[i]
      #shape = 1,4,2
      sup_left = (esq[(0,0,0)],esq[(0,0,1)])
      inf_rigth = (esq[(0,2,0)],esq[(0,2,1)])
      image = cv2.rectangle(cv_image, sup_left, inf_rigth, color, thickness)
  
  cv2.imshow("RGB Image", cv_image)
  cv2.waitKey(3)

  try:
    #position_pub.publish(piece_pose)
    fin = 1
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('image_sub', anonymous=True)

  global bridge, cv_depth, arr, img_bgr, position_pub, image_sub, depth_image_sub, depth_points_sub, depth_img, Red_mask
  print("Image Processing Node - Looking for piece")

  bridge = CvBridge()

  #position_pub      = rospy.Publisher("/redpiece_pos",PointStamped,queue_size=10)
  image_sub         = rospy.Subscriber("/camera/rgb/image_color",Image,callback_image)
  #depth_image_sub   = rospy.Subscriber("/camera/depth/image_raw",Image,callback_depth_image)
  #depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  
  cv_depth = np.zeros((480, 640))
  depth_img = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)