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

def move_distance(goal_dist, goal_angle, pub_goal_dist):
  msg_dist = Float32MultiArray()
  msg_dist.data = [goal_dist, goal_angle]
  print(msg_dist)
  pub_goal_dist.publish(msg_dist)


def callback_depth_points(data):
  global arr, img_bgr
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  rgb_arr = arr['rgb'].copy()
  rgb_arr.dtype = np.uint32
  r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
  img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
  

def callback_depth_image(data):
  global depth_img
  try:
    depth_img = bridge.imgmsg_to_cv2(data, "16UC1")
  except CvBridgeError as e:
    print(e)
  depth_img = depth_img*10
  

def callback_image(data):
  global depth_img, arr, img_bgr
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  tfBuffer = tf2_ros.Buffer()

  Masked_red = np.zeros((480, 640))

  piece_pose = PointStamped()
  piece_pose.header.stamp = rospy.Time.now()
  piece_pose.header.frame_id = "kinect_link"
  piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = 0,0,0


  img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

  delta_red = 115

  mean_red = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
  up_h_red = mean_red[0] + 10
  lw_h_red = mean_red[0] - 10
  up_s_red = mean_red[1] + 30
  lw_s_red = mean_red[1] - 30
  up_v_red = mean_red[2] + delta_red
  lw_v_red = mean_red[2] - delta_red
  up_red = (up_h_red,up_s_red,up_v_red,0.0)
  lw_red = (lw_h_red,lw_s_red,lw_v_red,0.0)
  Fil_red = cv2.inRange(img_hsv, lw_red, up_red)

  mean_red1 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
  up_h_red1 = mean_red1[0] + 10
  lw_h_red1 = mean_red1[0] - 10
  up_s_red1 = mean_red1[1] + 30
  lw_s_red1 = mean_red1[1] - 30
  up_v_red1 = mean_red1[2] + delta_red
  lw_v_red1 = mean_red1[2] - delta_red
  up_red1 = (up_h_red1,up_s_red1,up_v_red1,0.0)
  lw_red1 = (lw_h_red1,lw_s_red1,lw_v_red1,0.0)
  Fil_red1 = cv2.inRange(img_hsv, lw_red1, up_red1)

  Red_mask = cv2.bitwise_or(Fil_red, Fil_red1)

  mean_red2 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
  up_h_red2 = mean_red2[0] + 10
  lw_h_red2 = mean_red2[0] - 10
  up_s_red2 = mean_red2[1] + 30
  lw_s_red2 = mean_red2[1] - 30
  up_v_red2 = mean_red2[2] + delta_red
  lw_v_red2 = mean_red2[2] - delta_red
  up_red2 = (up_h_red2,up_s_red2,up_v_red2,0.0)
  lw_red2 = (lw_h_red2,lw_s_red2,lw_v_red2,0.0)
  Fil_red2 = cv2.inRange(img_hsv, lw_red2, up_red2)

  Red_mask = cv2.bitwise_or(Red_mask, Fil_red2)

  mean_red3 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)
  up_h_red3 = mean_red3[0] + 10
  lw_h_red3 = mean_red3[0] - 10
  up_s_red3 = mean_red3[1] + 30
  lw_s_red3 = mean_red3[1] - 30
  up_v_red3 = mean_red3[2] + delta_red
  lw_v_red3 = mean_red3[2] - delta_red
  up_red3 = (up_h_red3,up_s_red3,up_v_red3,0.0)
  lw_red3 = (lw_h_red3,lw_s_red3,lw_v_red3,0.0)
  Fil_red3 = cv2.inRange(img_hsv, lw_red3, up_red3)

  Red_mask = cv2.bitwise_or(Red_mask, Fil_red3)
  loc = cv2.findNonZero(Red_mask)

  #loc[i,0,0] : Points_i
  #loc[i,0,1] : Points_j
  for i in range(loc.shape[0]):
    pos_x = float(arr[loc[i,0,1],loc[i,0,0]][0])
    pos_y = float(arr[loc[i,0,1],loc[i,0,0]][1])
    pos_z = float(arr[loc[i,0,1],loc[i,0,0]][2])
    if math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z):
      nanis = 1
    else:
      piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
      #print(piece_pose.point,"\n")

  br = tf.TransformBroadcaster()
  rate = rospy.Rate(1.0)
  br.sendTransform((piece_pose.point.z, piece_pose.point.x, piece_pose.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "piece_rgbd_sensor_link", "kinect_link")
  
  listener = tf.TransformListener()
  listener.waitForTransform('/base_link', 'kinect_link', rospy.Time(), rospy.Duration(1.0))
  piece_pose2 = listener.transformPoint('base_link', piece_pose)

  #print(piece_pose2)
  


  Red_mask = cv2.medianBlur(Red_mask,9)
  M = cv2.moments(Red_mask)
  try: 
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(cv_image, "centroid_Red", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    Masked_red = cv2.bitwise_and(cv_image, cv_image, mask = Red_mask)
    cX_depth = cX
    cY_depth = cY
    cv2.circle(depth_img, (cX_depth, cY_depth), 5, (0, 255, 255), -1)
    cv2.putText(depth_img, "centroid_Red", (cX_depth - 25, cY_depth - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
  except ZeroDivisionError as e:
    print("Object not found")

  
  mean_blk = (73.37954475229168, 93.521268925739, 0.0, 0.0)
  delta_blk = 100
  up_h_blk = mean_blk[0] + delta_blk
  lw_h_blk = mean_blk[0] - delta_blk
  up_s_blk = mean_blk[1] + delta_blk
  lw_s_blk = mean_blk[1] - delta_blk
  up_v_blk = mean_blk[2] + delta_blk
  lw_v_blk = mean_blk[2] - delta_blk
  up_blk = (up_h_blk,up_s_blk,up_v_blk,0.0)
  lw_blk = (lw_h_blk,lw_s_blk,lw_v_blk,0.0)
  #print(lw_blk,"\n", up_blk)
  lw_blk = (0,0,0)
  up_blk = (360,255,50)
  Fil_blk = cv2.inRange(img_hsv, lw_blk, up_blk)
  Fil_blk = cv2.medianBlur(Fil_blk,9)
  """M = cv2.moments(Fil_blk)
  cX = int(M["m10"] / M["m00"])
  cY = int(M["m01"] / M["m00"])
  cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
  cv2.putText(cv_image, "centroid_Black", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
  """

  cv2.imshow("RGB Image", cv_image)

  cv2.imshow("Red Piece Mask", Red_mask)
  cv2.waitKey(3)

  try:
    position_pub.publish(piece_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('image_sub', anonymous=True)

  global bridge, cv_depth, arr, img_bgr, position_pub, image_sub, depth_image_sub, depth_points_sub
  print("Image Processing Node - Looking for piece")

  bridge = CvBridge()

  position_pub      = rospy.Publisher("/redpiece_pos",PointStamped,queue_size=10)
  image_sub         = rospy.Subscriber("/camera/rgb/image_color",Image,callback_image)
  depth_image_sub   = rospy.Subscriber("/camera/depth/image_raw",Image,callback_depth_image)
  depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  
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