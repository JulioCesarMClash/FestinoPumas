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
from img_proc.srv import *

import roslib
roslib.load_manifest('img_proc')

import yaml
from yaml.loader import SafeLoader

bridge = CvBridge()

def segment_color(img_bgr, hsv_mean):
  img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
  img_h, img_s, img_v = cv2.split(img_hsv)
  #cv2.imwrite('H.png', img_h)
  #cv2.imwrite('S.png', img_s)
  #cv2.imwrite('V.png', img_v)
  delta_h = 10
  delta_s = 15
  delta_v = 70
  up_h = hsv_mean[0] + delta_h
  lw_h = hsv_mean[0] - delta_h
  up_s = hsv_mean[1] + delta_s
  lw_s = hsv_mean[1] - delta_s
  up_v = hsv_mean[2] + delta_v
  lw_v = hsv_mean[2] - delta_v
  up = (up_h, up_s, up_v, 0.0)
  lw = (lw_h, lw_s, lw_v, 0.0)
  Fil = cv2.inRange(img_hsv, lw, up)
  return Fil

def callback_depth_points(data):
  global arr
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)

def load_means_data_asarg():
  piece = 0
  with open('/home/robocup/FestinoPumas/PC_user/src/Vision/img_proc/scripts/means_SolidColor_Pieces.yaml', 'r') as f:
    data = list(yaml.load_all(f, Loader=SafeLoader))
  if len(sys.argv) > 1:
    for i in data:
      if sys.argv[1] == i['PieceColor']:
        piece = i['PieceCode']
        print("Looking for:", i['PieceColor'], "Piece")
  else: 
    print("No recibi argumento, buscare la pieza roja c:")
  PieceInfo = data[piece]
  return PieceInfo

def load_means_data_as_srv(piece):
  print(piece)
  piece_num = 0
  with open('/home/robocup/FestinoPumas/PC_user/src/Vision/img_proc/scripts/means_SolidColor_Pieces.yaml', 'r') as f:
    data = list(yaml.load_all(f, Loader=SafeLoader))
  if piece is not None:
    for i in data:
      if piece == i['PieceColor']:
        piece_num = i['PieceCode']
        print("Looking for:", i['PieceColor'], "Piece")
  else: 
    print("No recibi argumento, buscare la pieza roja c:")
  PieceInfo = data[piece_num]
  return PieceInfo

def find_piece(request):
  response = Find_piece_SrvResponse()
  response.success = False
  global arr, img_bgr, mask, rate, position_pub,pub_vel

  

  vel = Twist()
  vel.linear.x = 0
  vel.linear.y = 0
  vel.angular.z = 0
  ######## Imaginary MPS tf 1 ########

  mps_statictf = geometry_msgs.msg.TransformStamped()

  mps_statictf.header.stamp = rospy.Time.now()
  mps_statictf.header.frame_id = "camera_link"
  mps_statictf.child_frame_id = "MPS_1"

  mps_statictf.transform.rotation.x = 0.0
  mps_statictf.transform.rotation.y = 0.0
  mps_statictf.transform.rotation.z = 0.0
  mps_statictf.transform.rotation.w = 1.0

  mps_statictf.transform.translation.x = 3.5
  mps_statictf.transform.translation.y = -4.5
  mps_statictf.transform.translation.z = 0.0
  mps_static_br = tf2_ros.StaticTransformBroadcaster()
  mps_static_br.sendTransform(mps_statictf)

  ######## Imaginary MPS tf 2 ########

  ######## Filling msg for tapita_pose publisher 1 ########
  piece_pose = PointStamped()
  piece_pose.header.stamp = rospy.Time.now()
  piece_pose.header.frame_id = "camera_link"
  piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = 0, 0, 0
  ######## Filling msg for tapita_pose publisher 2 ########

  ######## Filling msg for tapita_static_pose publisher ########

  static_transformStamped = geometry_msgs.msg.TransformStamped()

  static_transformStamped.header.stamp = rospy.Time.now()
  static_transformStamped.header.frame_id = "camera_link"
  static_transformStamped.child_frame_id = "piece_static_link"

  static_transformStamped.transform.rotation.x = 0.0
  static_transformStamped.transform.rotation.y = 0.0
  static_transformStamped.transform.rotation.z = 0.0
  static_transformStamped.transform.rotation.w = 1.0

  PieceInfo = load_means_data_as_srv(request.piece)

  tapita_found = False
  while tapita_found is not True:
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r, g, b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((np.asarray(b, dtype='uint8'), np.asarray(
        g, dtype='uint8'), np.asarray(r, dtype='uint8')))
    Masked = np.zeros((480, 640))
    img_bgr_display = img_bgr.copy()
    img_bgr[0:240,0:640,:]=np.zeros((240,640,3),np.uint8)

    ######## Looking for Piece ########

    Fil_l1 = segment_color(img_bgr, tuple([float(i) for i in PieceInfo['Mean_L1'].split(',')]))
    Fil_l2 = segment_color(img_bgr, tuple([float(i) for i in PieceInfo['Mean_L2'].split(',')]))
    Fil_l3 = segment_color(img_bgr, tuple([float(i) for i in PieceInfo['Mean_L3'].split(',')]))
    Fil_l4 = segment_color(img_bgr, tuple([float(i) for i in PieceInfo['Mean_L4'].split(',')]))

    #print("Filtered Image obtained")

    mask1 = cv2.bitwise_or(Fil_l1, Fil_l2)
    mask2 = cv2.bitwise_or(mask1, Fil_l3)
    mask3 = cv2.bitwise_or(mask2, Fil_l4)

    kernel_E = np.ones((5, 5), np.uint8)
    kernel_D = np.ones((3, 3), np.uint8)

    img_erosion = cv2.erode(mask3, kernel_E, iterations=1) 
    mask = cv2.dilate(img_erosion, kernel_D, iterations=1) 

    #print("Mask obtained")

    mask = cv2.medianBlur(mask, 9)
    loc = cv2.findNonZero(mask)

    #print("NonZeros found")

    # loc[i,0,0] : Points_i
    # loc[i,0,1] : Points_j
    try:
      for i in range(loc.shape[0]):
        pos_x = float(arr[loc[i, 0, 1], loc[i, 0, 0]][2])
        pos_y = -float(arr[loc[i, 0, 1], loc[i, 0, 0]][0])-0.05
        pos_z = -float(arr[loc[i, 0, 1], loc[i, 0, 0]][1])

        if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
          #if not (pos_x > 1.0 or abs(pos_y) > 0.2 or pos_z > 0.3):
          static_transformStamped.transform.translation.x = pos_x
          static_transformStamped.transform.translation.y = pos_y
          static_transformStamped.transform.translation.z = pos_z
          piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
          #br = tf.TransformBroadcaster()
          static_br = tf2_ros.StaticTransformBroadcaster()
          #br.sendTransform((piece_pose.point.x, piece_pose.point.y, piece_pose.point.z),(0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "piece_link", "camera_link")
          static_br.sendTransform(static_transformStamped)

          M = cv2.moments(mask)
          try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img_bgr_display, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img_bgr_display, "piece_centroid", (cX - 25, cY - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            Masked = cv2.bitwise_and(img_bgr_display, img_bgr_display, mask=mask)
            #print('Tapita Found')
            tapita_found = True
            print(piece_pose.point)
            response.success = True
            response.point_stamped = piece_pose
          except ZeroDivisionError as e:
            print("Object not found")
          else:
            print("Coordenadas inadecuadas")
        else:
          print("Nans")
    except AttributeError:
      #print('NoVeoNoVeo')
      response.success = False
      response.point_stamped = piece_pose
    #print(piece_pose.point)
    #cv2.imshow("Image", img_bgr_display)
    #cv2.imshow("Mask", mask)
    cv2.waitKey(3)
  return response

  try:
    position_pub.publish(piece_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('image_sub', anonymous=True)

  global cv_depth, arr, img_bgr, position_pub, image_sub
  global depth_image_sub, depth_points_sub, mask, rate, position_pub, pub_vel
  rate = rospy.Rate(10.0)

  print("Image Processing Node - Looking for piece")

  position_pub        = rospy.Publisher("/piece_pos",PointStamped,queue_size=10)
  pub_vel             = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
  depth_points_sub    = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  find_piece_service  = rospy.Service('/vision/find_piece/point_stamped', Find_piece_Srv, find_piece)
  
  cv_depth = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
