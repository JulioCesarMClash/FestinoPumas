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

def callback_aruco(data):
  try:
    image_aruco = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
  parameters = cv2.aruco.DetectorParameters_create()
  markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(image_aruco, dictionary, parameters=parameters)
  color = (255, 0, 0)
  thickness = 2
  if(markerIds.shape[0] > 1):
    for i in range (markerIds.shape[0]):
      esq = markerCorners[i]
      #shape = 1,4,2
      sup_left = (esq[(0,0,0)],esq[(0,0,1)])
      inf_rigth = (esq[(0,2,0)],esq[(0,2,1)])
      image = cv2.rectangle(image_aruco, sup_left, inf_rigth, color, thickness)
  
  cv2.imshow("Aruco Tags", image_aruco)
  cv2.waitKey(3)

  try:
    #position_pub.publish(piece_pose)
    fin = 1
  except CvBridgeError as e:
    print(e)


def callback_depth_points(data):
  global arr, img_bgr, Red_mask
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  rgb_arr = arr['rgb'].copy()
  rgb_arr.dtype = np.uint32
  r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
  img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))

  # Medias de color en hsv de la pieza en distintas iluminaciones
  mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
  mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
  mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
  mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

  Fil_R_l1 = segment_color(img_bgr,arr,mean_R_l1)
  Fil_R_l2 = segment_color(img_bgr,arr,mean_R_l2)
  Fil_R_l3 = segment_color(img_bgr,arr,mean_R_l3)
  Fil_R_l4 = segment_color(img_bgr,arr,mean_R_l4)

  Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
  Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
  Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
  Red_mask = cv2.medianBlur(Red_mask3,9)

def callback_depth_image(data):
  global depth_img
  try:
    depth_img = bridge.imgmsg_to_cv2(data, "16UC1")
  except CvBridgeError as e:
    print(e)
  depth_img = depth_img*10
  
def segment_color(img_bgr, img_xyz, hsv_mean):
  img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
  delta = 115
  up_h = hsv_mean[0] + 10
  lw_h = hsv_mean[0] - 10
  up_s = hsv_mean[1] + 30
  lw_s = hsv_mean[1] - 30
  up_v = hsv_mean[2] + delta
  lw_v = hsv_mean[2] - delta
  up = (up_h,up_s,up_v,0.0)
  lw = (lw_h,lw_s,lw_v,0.0)
  Fil = cv2.inRange(img_hsv, lw, up)
  return Fil

def callback_image(data):
  global depth_img, arr, img_bgr, Red_mask
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  ######## Filling msg for tapita_pose publisher ########  
  piece_pose = PointStamped()
  piece_pose.header.stamp = rospy.Time.now()
  piece_pose.header.frame_id = "kinect_link"
  piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = 0,0,0

  ######## Looking for ARUCO TAG ########
  image_aruco = cv_image
  dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
  parameters = cv2.aruco.DetectorParameters_create()
  markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(image_aruco, dictionary, parameters=parameters)
  color = (255, 0, 0)
  thickness = 2
  if(markerIds.shape[0] >= 1):
    for i in range (markerIds.shape[0]):
      esq = markerCorners[i]
      #shape = 1,4,2
      sup_left = (esq[(0,0,0)],esq[(0,0,1)])
      inf_rigth = (esq[(0,2,0)],esq[(0,2,1)])
      image = cv2.rectangle(image_aruco, sup_left, inf_rigth, color, thickness)
      if markerIds[i] == 13:
        print("MPS_1 ------------- Looking for piece")

    ######## Looking for ARUCO TAG ########
    tfBuffer = tf2_ros.Buffer()

    Masked_red = np.zeros((480, 640))

    

    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # --Listo-- utilizar la funcion para obtener las medias (en vez de tanto bloque de texto) 
    # --Listo-- (puede ser dentro del callback de la nube de puntos)
    # definir un archivo (yaml o txt o algo) donde se definan los parametros que se usaran por objeto (en este caso las medias de color)
    # imprimor cuatos pixeles entran en cada intervalo de color y buscar el umbral minimo que corresponderia a la pieza
    # eliminar pixeles sueltos si estan fuera de la mancha (con erode)
    # quitar el medianBlur - erode
    # migrar a Juskeshino 
    # atiende a servicio que pasa la informacion necesaria y me devuelve coordenada de la pieza

    loc = cv2.findNonZero(Red_mask)

    #loc[i,0,0] : Points_i
    #loc[i,0,1] : Points_j
    for i in range(loc.shape[0]):
      pos_x = float(arr[loc[i,0,1],loc[i,0,0]][0])
      pos_y = float(arr[loc[i,0,1],loc[i,0,0]][1])
      pos_z = float(arr[loc[i,0,1],loc[i,0,0]][2])
      #-Sacar la media de los puntos
      if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
        piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
        #print(piece_pose.point,"\n")

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1.0)
    br.sendTransform((piece_pose.point.z, piece_pose.point.x, piece_pose.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "piece_rgbd_sensor_link", "camera_link")
    #br.sendTransform((piece_pose.point.z, -piece_pose.point.x, -piece_pose.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "piece_rgbd_sensor_link", "camera_link")
    
    #listener = tf.TransformListener()
    #listener.waitForTransform('/base_link', 'kinect_link', rospy.Time(), rospy.Duration(1.0))
    #piece_pose2 = listener.transformPoint('base_link', piece_pose)

    #print(piece_pose2)
    
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
  else:
    print('Unable to find Tapita')

  cv2.imshow("Aruco Tags", image_aruco)

  
  cv2.waitKey(3)

  try:
    position_pub.publish(piece_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('image_sub', anonymous=True)

  global bridge, cv_depth, arr, img_bgr, position_pub, image_sub, depth_image_sub, depth_points_sub, depth_img, Red_mask
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