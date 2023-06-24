#!/usr/bin/env python
from __future__ import print_function

import roslib
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

#Festino dep
from img_proc.srv import *

roslib.load_manifest('img_proc')

class FindTagNode:
  def __init__(self):
    global arr
    rospy.init_node('find_tag_service')
    self.bridge = CvBridge()
    arr = np.zeros((480, 640))

    global depth_img_bgr, aruco_pos_pub, depth_points_sub, mps_name_pub
    print("Image Processing Srv - Looking for Tag")
    rate = rospy.Rate(10)
    self.bridge = CvBridge()

    self.depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.callback_depth_points)
    self.aruco_pos_pub     = rospy.Publisher("/aruco_pos",PointStamped,queue_size=10)

    self.find_tag_service = rospy.Service('/vision/find_tag/point_stamped', Find_tag_Srv, self.find_tag)

  def callback_depth_points(self, data):
    global arr
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    return

  def spin(self):
    rospy.spin()

  def find_tag(self, request):
    global arr, depth_img_bgr, aruco_pos_pub, depth_points_sub, mps_name_pub
    name_list = []
    aruco_list = PointStamped()
    aruco_list = []
    if request.is_find_tag_enabled:
      tfBuffer = tf2_ros.Buffer()
      depth_img_bgr = np.zeros((480, 640))
      mps_name = [0,0]
      rgb_arr = arr['rgb'].copy()
      rgb_arr.dtype = np.uint32
      r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
      aruco_img = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
      ######## Filling msg for aruco_pose publisher ########
      frame_id = "camera_link"
      aruco_pose = PointStamped()
      aruco_pose.header.stamp = rospy.Time.now()
      aruco_pose.header.frame_id = frame_id
      aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = 0,0,0

      ######## Looking for ARUCO TAG ########
      dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
      parameters = cv2.aruco.DetectorParameters_create()
      markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(aruco_img, dictionary, parameters=parameters)

      color = (255, 0, 0)
      thickness = 2
      aruco_det_flag = False
      mps_name = "Not Identified"
      try:
        if(markerIds.shape[0] >= 1):
          for i in range (markerIds.shape[0]):
            aruco_pose_aux = PointStamped()
            corners = markerCorners[i]
            #shape = 1,4,2
            first_corner = (corners[(0,0,0)],corners[(0,0,1)])
            last_corner = (corners[(0,2,0)],corners[(0,2,1)])
                
            aruco_img = cv2.rectangle(aruco_img, first_corner, last_corner, color, thickness)
            known_markers = ([101,102,103,104,111,112,113,114,121,122,131,132,141,142,201,202,203,204,211,212,213,214,221,222,231,232,241,242])

            if markerIds[i] in known_markers:
              aruco_det_flag = True
              max_x = np.max([last_corner[0],first_corner[0]])
              min_x = np.min([last_corner[0],first_corner[0]])

              max_y = np.max([last_corner[1],first_corner[1]])
              min_y = np.min([last_corner[1],first_corner[1]])
              
              cent = (int(max_x - (max_x-min_x)/2),int(max_y - (max_y-min_y)/2))
              aruco_img = cv2.rectangle(aruco_img, first_corner, last_corner, (255, 255, 0), thickness)
              cv2.circle(aruco_img, (cent), 5, (0, 255, 255), -1)
              try:
                pos_x = float(arr[cent][0])
                pos_y = float(arr[cent][1])
                pos_z = float(arr[cent][2])
                mps_name_arr, mps_name = aruco_mps(markerIds[i])
                #print(mps_name, "\n")
                #self.mps_name_pub.publish(mps_name)

                if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
                    aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = pos_z, -pos_y, -pos_x
                    br_ar = tf.TransformBroadcaster()
                    br_ar.sendTransform((aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), mps_name, frame_id)
                    aruco_pose_aux.point.x = aruco_pose.point.x
                    aruco_pose_aux.point.y = aruco_pose.point.y
                    aruco_pose_aux.point.z = aruco_pose.point.z
                    name_list.append(mps_name)
                    aruco_list.append(aruco_pose_aux)
                    print(aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z, '\n')
                    #self.aruco_pos_pub.publish(aruco_pose)
                    #print(aruco_pose)
                    #print(aruco_pose_aux)

                
              except IndexError:
                print('Not identified')
      except AttributeError:
        print('No Tag')
      print(name_list)
      print(aruco_list)
      response = Find_tag_SrvResponse()
      response.success = True
      response.mps_name = name_list
      response.point_stamped = aruco_list

      return response
    
    else:

      response = Find_tag_SrvResponse()
      response.success = False
      aruco_pose_fake = PointStamped()
      aruco_pose_fake.point.x = 0.0
      aruco_pose_fake.point.y = 0.0
      aruco_pose_fake.point.z = 0.0
      response.point_stamped.append(aruco_pose_fake)
      name = " "
      response.mps_name.append(name)  
      return response

def aruco_mps(aruco_id):
  ######## CYAN STATIONS ########
  #CapStations
  mps_name_arr = 0
  if(aruco_id == 101):
    mps_name_arr = ['C','CS1','O']
  elif(aruco_id == 102):
    mps_name_arr = ['C','CS1','I']
  elif(aruco_id == 103):
    mps_name_arr = ['C','CS2','O']
  elif(aruco_id == 104):
    mps_name_arr = ['C','CS2','I']

  #RingStations
  elif(aruco_id == 111):
    mps_name_arr = ['C','RS1','O']
  elif(aruco_id == 112):
    mps_name_arr = ['C','RS1','I']
  elif(aruco_id == 113):
    mps_name_arr = ['C','RS2','O']
  elif(aruco_id == 114):
    mps_name_arr = ['C','RS2','I']

  #BaseStations
  elif(aruco_id == 121):
    mps_name_arr = ['C','BS','O']
  elif(aruco_id == 122):
    mps_name_arr = ['C','BS','I']

  #DeliveryStations
  elif(aruco_id == 131):
    mps_name_arr = ['C','DS','O']
  elif(aruco_id == 132):
    mps_name_arr = ['C','DS','I']
  
  #StorageStations
  elif(aruco_id == 141):
    mps_name_arr = ['C','SS','O']
  elif(aruco_id == 142):
    mps_name_arr = ['C','SS','I']

  ######## MAGENTA STATIONS ########
  #CapStations
  elif(aruco_id == 201):
    mps_name_arr = ['M','CS1','O']
  elif(aruco_id == 202):
    mps_name_arr = ['M','CS1','I']
  elif(aruco_id == 203):
    mps_name_arr = ['M','CS2','O']
  elif(aruco_id == 204):
    mps_name_arr = ['M','CS2','I']

  #RingStations
  elif(aruco_id == 211):
    mps_name_arr = ['M','RS1','O']
  elif(aruco_id == 212):
    mps_name_arr = ['M','RS1','I']
  elif(aruco_id == 213):
    mps_name_arr = ['M','RS2','O']
  elif(aruco_id == 214):
    mps_name_arr = ['M','RS2','I']

  #BaseStations
  elif(aruco_id == 221):
    mps_name_arr = ['M','BS','O']
  elif(aruco_id == 222):
    mps_name_arr = ['M','BS','I']

  #DeliveryStations
  elif(aruco_id == 231):
    mps_name_arr = ['M','DS','O']
  elif(aruco_id == 232):
    mps_name_arr = ['M','DS','I']
  
  #StorageStations
  elif(aruco_id == 241):
    mps_name_arr = ['M','SS','O']
  elif(aruco_id == 242):
    mps_name_arr = ['M','SS','I']
  mps_name = mps_name_arr[0]+'-'+mps_name_arr[1]+'-'+mps_name_arr[2]
  return mps_name_arr, mps_name 

if __name__ == '__main__':
  find_tag_node = FindTagNode()
  find_tag_node.spin()