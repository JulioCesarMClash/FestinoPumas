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

tfBuffer = tf2_ros.Buffer()
bridge = CvBridge()

def aruco_mps(aruco_id):
  ######## CYAN STATIONS ########
  #CapStations
  mps_name_arr = 0
  if(aruco_id == 101):
    mps_name_arr = ['C','CS1','Output']
  elif(aruco_id == 102):
    mps_name_arr = ['C','CS1','Input']
  elif(aruco_id == 103):
    mps_name_arr = ['C','CS2','Output']
  elif(aruco_id == 104):
    mps_name_arr = ['C','CS2','Input']

  #RingStations
  elif(aruco_id == 111):
    mps_name_arr = ['C','RS1','Output']
  elif(aruco_id == 112):
    mps_name_arr = ['C','RS1','Input']
  elif(aruco_id == 113):
    mps_name_arr = ['C','RS2','Output']
  elif(aruco_id == 114):
    mps_name_arr = ['C','RS2','Input']

  #BaseStations
  elif(aruco_id == 121):
    mps_name_arr = ['C','BS','Output']
  elif(aruco_id == 122):
    mps_name_arr = ['C','BS','Input']

  #DeliveryStations
  elif(aruco_id == 131):
    mps_name_arr = ['C','DS','Output']
  elif(aruco_id == 132):
    mps_name_arr = ['C','DS','Input']
  
  #StorageStations
  elif(aruco_id == 141):
    mps_name_arr = ['C','SS','Output']
  elif(aruco_id == 142):
    mps_name_arr = ['C','SS','Input']

  ######## MAGENTA STATIONS ########
  #CapStations
  elif(aruco_id == 201):
    mps_name_arr = ['M','CS1','Output']
  elif(aruco_id == 202):
    mps_name_arr = ['M','CS1','Input']
  elif(aruco_id == 203):
    mps_name_arr = ['M','CS2','Output']
  elif(aruco_id == 204):
    mps_name_arr = ['M','CS2','Input']

  #RingStations
  elif(aruco_id == 211):
    mps_name_arr = ['M','RS1','Output']
  elif(aruco_id == 212):
    mps_name_arr = ['M','RS1','Input']
  elif(aruco_id == 213):
    mps_name_arr = ['M','RS2','Output']
  elif(aruco_id == 214):
    mps_name_arr = ['M','RS2','Input']

  #BaseStations
  elif(aruco_id == 221):
    mps_name_arr = ['M','BS','Output']
  elif(aruco_id == 222):
    mps_name_arr = ['M','BS','Input']

  #DeliveryStations
  elif(aruco_id == 231):
    mps_name_arr = ['M','DS','Output']
  elif(aruco_id == 232):
    mps_name_arr = ['M','DS','Input']
  
  #StorageStations
  elif(aruco_id == 241):
    mps_name_arr = ['M','SS','Output']
  elif(aruco_id == 242):
    mps_name_arr = ['M','SS','Input']
  mps_name = mps_name_arr[0]+'-'+mps_name_arr[1]+'---'+mps_name_arr[2]
  return mps_name_arr, mps_name

def callback_depth_points(data):
  global rate, arr
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  mps_name = [0,0]
  rgb_arr = arr['rgb'].copy()
  rgb_arr.dtype = np.uint32
  r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
  aruco_img = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))

  ######## Filling msg for aruco_pose publisher ########  
  aruco_pose = PointStamped()
  aruco_pose.header.stamp = rospy.Time.now()
  aruco_pose.header.frame_id = "kinect_link"
  aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = 0,0,0

  ######## Looking for ARUCO TAG ########
  dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
  parameters = cv2.aruco.DetectorParameters_create()
  markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(aruco_img, dictionary, parameters=parameters)

  #print('Looking for ARUCO tags')

  color = (255, 0, 0)
  thickness = 2
  if(markerIds.shape[0] >= 1):
    for i in range (markerIds.shape[0]):
      corners = markerCorners[i]
      #shape = 1,4,2
      first_corner = (corners[(0,0,0)],corners[(0,0,1)])
      last_corner = (corners[(0,2,0)],corners[(0,2,1)])
          
      aruco_img = cv2.rectangle(aruco_img, first_corner, last_corner, color, thickness)

      #if (101 or 102 or 103 or 104 or 111 or 112 or 113 or 114 or 121 or 122 or 131 or 132 or 141 or 142 or 201 or 202 or 203 or 204 or 211 or 212 or 213 or 214 or 221 or 222 or 231 or 232 or 241 or 242) in markerIds:
      if (markerIds[i] == 101) or (markerIds[i] == 102) or (markerIds[i] == 103) or (markerIds[i] == 104) or (markerIds[i] == 111) or (markerIds[i] == 112) or (markerIds[i] == 113) or (markerIds[i] == 114) or (markerIds[i] == 121) or (markerIds[i] == 122) or (markerIds[i] == 131) or (markerIds[i] == 132) or (markerIds[i] == 141) or (markerIds[i] == 142) or (markerIds[i] == 201) or (markerIds[i] == 202) or (markerIds[i] == 203) or (markerIds[i] == 204) or (markerIds[i] == 211) or (markerIds[i] == 212) or (markerIds[i] == 213) or (markerIds[i] == 214) or (markerIds[i] == 221) or (markerIds[i] == 222) or (markerIds[i] == 231) or (markerIds[i] == 232) or (markerIds[i] == 241) or (markerIds[i] == 242):
    
        max_x = np.max([last_corner[0],first_corner[0]])
        min_x = np.min([last_corner[0],first_corner[0]])

        max_y = np.max([last_corner[1],first_corner[1]])
        min_y = np.min([last_corner[1],first_corner[1]])
        
        cent = (int(max_x - (max_x-min_x)/2),int(max_y - (max_y-min_y)/2))
        aruco_img = cv2.rectangle(aruco_img, first_corner, last_corner, (255, 255, 0), thickness)
        cv2.circle(aruco_img, (cent), 5, (0, 255, 255), -1)
        pos_x = float(arr[cent][0])
        pos_y = float(arr[cent][1])
        pos_z = float(arr[cent][2])
        mps_name_arr, mps_name = aruco_mps(markerIds[i])
        print(mps_name)

        if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
            aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = pos_x, pos_y, pos_z
            br_ar = tf.TransformBroadcaster()
            br_ar.sendTransform((aruco_pose.point.z, -aruco_pose.point.x, -aruco_pose.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), mps_name, "camera_link")
  cv2.imshow("Aruco Tags", aruco_img)
  cv2.waitKey(3)

  try:
    aruco_pos_pub.publish(aruco_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('aruco_det', anonymous=True)

  global rate, arr, depth_img_bgr, aruco_pos_pub, depth_points_sub, msg_pub
  print("Image Processing Node - Looking for piece")
  rate = rospy.Rate(0.1)
  depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  aruco_pos_pub     = rospy.Publisher("/aruco_pos",PointStamped,queue_size=10)
  msg_pub           = rospy.Publisher("/speak", String,queue_size=1)
  msg = String()
  
  depth_img_bgr = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)