#!/usr/bin/env python
from __future__ import print_function
from vision_msgs.srv import RecognizeObjects
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
import cv2
import tf2_ros
import tf_conversions
import tf
import math
import numpy as np
import ros_numpy
import rospy
import sys

import roslib
roslib.load_manifest('img_proc')

bridge = CvBridge()

def segment_color(img_bgr, img_xyz, hsv_mean):
  img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
  delta = 115
  up_h = hsv_mean[0] + 10
  lw_h = hsv_mean[0] - 10
  up_s = hsv_mean[1] + 30
  lw_s = hsv_mean[1] - 30
  up_v = hsv_mean[2] + delta
  lw_v = hsv_mean[2] - delta
  up = (up_h, up_s, up_v, 0.0)
  lw = (lw_h, lw_s, lw_v, 0.0)
  Fil = cv2.inRange(img_hsv, lw, up)
  return Fil

def callback_depth_points(data):
  global arr, img_bgr, Red_mask, rate
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  rgb_arr = arr['rgb'].copy()
  rgb_arr.dtype = np.uint32
  r, g, b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
  img_bgr = cv2.merge((np.asarray(b, dtype='uint8'), np.asarray(
      g, dtype='uint8'), np.asarray(r, dtype='uint8')))

  # Medias de color en hsv de la pieza en distintas iluminaciones
  mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
  mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
  mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
  mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

  Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
  Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
  Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
  Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

  Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
  Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
  Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
  Red_mask = cv2.medianBlur(Red_mask3, 9)

  ######## Filling msg for tapita_pose publisher ########
  piece_pose = PointStamped()
  piece_pose.header.stamp = rospy.Time.now()
  piece_pose.header.frame_id = "camera_link"
  piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = 0, 0, 0

  aruco_pose = piece_pose

  ######## Filling msg for tapita_static_pose publisher ########

  static_transformStamped = geometry_msgs.msg.TransformStamped()

  static_transformStamped.header.stamp = rospy.Time.now()
  static_transformStamped.header.frame_id = "map"
  static_transformStamped.child_frame_id = "piece_static_link"

  static_transformStamped.transform.rotation.x = 0.0
  static_transformStamped.transform.rotation.y = 0.0
  static_transformStamped.transform.rotation.z = 0.0
  static_transformStamped.transform.rotation.w = 1.0

  ######## Looking for Piece ########

  Masked_red = np.zeros((480, 640))
  img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
  loc = cv2.findNonZero(Red_mask)

  # loc[i,0,0] : Points_i
  # loc[i,0,1] : Points_j
  for i in range(loc.shape[0]):
    pos_x = float(arr[loc[i, 0, 1], loc[i, 0, 0]][0])
    pos_y = float(arr[loc[i, 0, 1], loc[i, 0, 0]][1])
    pos_z = float(arr[loc[i, 0, 1], loc[i, 0, 0]][2])

    static_transformStamped.transform.translation.x = pos_z
    static_transformStamped.transform.translation.y = -pos_x
    static_transformStamped.transform.translation.z = -pos_y
    # -Sacar la media de los puntos
    if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
      piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
      br = tf.TransformBroadcaster()
      static_br = tf2_ros.StaticTransformBroadcaster()
      br.sendTransform((piece_pose.point.z, -piece_pose.point.x, -piece_pose.point.y),
                       (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "piece_link", "camera_link")
      static_br.sendTransform(static_transformStamped)

      M = cv2.moments(Red_mask)
      try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(img_bgr, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(img_bgr, "centroid_Red", (cX - 25, cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        Masked_red = cv2.bitwise_and(img_bgr, img_bgr, mask=Red_mask)
        print('Tapita Found')
      except ZeroDivisionError as e:
        print("Object not found")

      mean_B_l1 = (73.37954475229168, 93.521268925739, 0.0, 0.0)
      Fil_B_l1 = segment_color(img_bgr, arr, mean_B_l1)
    else:
      print('Unable to find Tapita')
  cv2.imshow("Image", img_bgr)
  cv2.imshow("Red Piece Mask", Red_mask)
  cv2.waitKey(3)

  try:
    position_pub.publish(piece_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('image_sub', anonymous=True)

  global cv_depth, arr, img_bgr, position_pub, image_sub, depth_image_sub, depth_points_sub, Red_mask, rate
  rate = rospy.Rate(10.0)

  print("Image Processing Node - Looking for piece")

  position_pub      = rospy.Publisher("/piece_pos",PointStamped,queue_size=10)
  depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  
  cv_depth = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)