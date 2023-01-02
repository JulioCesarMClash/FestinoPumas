#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('img_proc')
import sys
import rospy
import cv2
import ros_numpy
import numpy as np
import math
import tf
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

## Haz una media de medias de diferentes iluminaciones del cilindrito
## Listo --- No uses un centroide, sino toda la mancha de color
## Listo --- Identifica las coordenadas pixel de todos los pixeles que pertenezcan
## al cilindrito 

class image_converter:

  def __init__(self):
    self.position_pub = rospy.Publisher("redpiece_pos",Point,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback_image)
    self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback_depth_image)
    self.depth_points_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.callback_depth_points)

  def callback_depth_points(self,data):
    global arr, img_bgr
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    

  def callback_depth_image(self,data):
    global depth_img
    try:
      depth_img = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e)
    depth_img = depth_img*10
    

  def callback_image(self,data):
    global depth_img, arr, img_bgr
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    Masked_red = np.zeros((480, 640))

    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    mean_red = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
    delta_red = 115
    up_h_red = mean_red[0] + 10
    lw_h_red = mean_red[0] - 10
    up_s_red = mean_red[1] + 30
    lw_s_red = mean_red[1] - 30
    up_v_red = mean_red[2] + delta_red
    lw_v_red = mean_red[2] - delta_red
    up_red = (up_h_red,up_s_red,up_v_red,0.0)
    lw_red = (lw_h_red,lw_s_red,lw_v_red,0.0)
    Fil_red = cv2.inRange(img_hsv, lw_red, up_red)
    loc = cv2.findNonZero(Fil_red)
    #loc[i,0,0] : Points_i
    #loc[i,0,1] : Points_j
    for i in range(loc.shape[0]):
      pos_x = float(arr[loc[i,0,1],loc[i,0,0]][0])
      pos_y = float(arr[loc[i,0,1],loc[i,0,0]][1])
      pos_z = float(arr[loc[i,0,1],loc[i,0,0]][2])
      if math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z):
        nanis = 1
      else:
        pos = (pos_x,pos_y,pos_z)
        print(pos)
        piece_pose = PointStamped()
        piece_pose.header.frame_id = "piece_rgbd_sensor_link"
        piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
        listener = tf.TransformListener()
        listener.waitForTransform("/base_link","piece_rgbd_sensor_link", rospy.Time(),rospy.Duration(1.0))
        piece_pose = listener.transformPoint("/base_link",piece_pose)
        print(piece_pose.point)


    Fil_red = cv2.medianBlur(Fil_red,5)
    M = cv2.moments(Fil_red)
    try: 
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
      cv2.putText(cv_image, "centroid_Red", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
      Masked_red = cv2.bitwise_and(cv_image, cv_image, mask = Fil_red)
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
    cv2.imshow("Depth Image", depth_img)
    cv2.imshow("Reconstructed from PointCloud", img_bgr)
    cv2.imshow("Only Red figure", Masked_red)
    cv2.waitKey(3)

    try:
      self.position_pub.publish(piece_pose)
    except CvBridgeError as e:
      print(e)

def main(args):
  print("Image Processing Node - Looking for piece")
  global cv_depth, arr, img_bgr
  cv_depth = np.zeros((480, 640))
  depth_img = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)