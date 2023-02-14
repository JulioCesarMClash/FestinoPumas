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
rate = rospy.Rate(1.0)

def callback_depth_points(data):
  global arr, depth,img_bgr
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  rgb_arr = arr['rgb'].copy()
  rgb_arr.dtype = np.uint32
  r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
  depth_img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))

def callback_image(data):
  global arr, depth_img_bgr
  try:
    aruco_img = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  ######## Filling msg for aruco_pose publisher ########  
  aruco_pose = PointStamped()
  aruco_pose.header.stamp = rospy.Time.now()
  aruco_pose.header.frame_id = "kinect_link"
  aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = 0,0,0

  ######## Looking for ARUCO TAG ########
  dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
  parameters = cv2.aruco.DetectorParameters_create()
  markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(aruco_img, dictionary, parameters=parameters)

  msg = 'I am lokking for ARUCO tags'
  msg_pub.publish(msg)
  rospy.sleep(2)
  
  color = (255, 0, 0)
  thickness = 2
  if(markerIds.shape[0] >= 1):
    for i in range (markerIds.shape[0]):
      corners = markerCorners[i]
      #shape = 1,4,2
      first_corner = (corners[(0,0,0)],corners[(0,0,1)])
      last_corner = (corners[(0,2,0)],corners[(0,2,1)])
          
      aruco_img = cv2.rectangle(aruco_img, first_corner, last_corner, color, thickness)

      if (markerIds[i] == 101) or (markerIds[i] == 102) or (markerIds[i] == 103) or (markerIds[i] == 104) or (markerIds[i] == 111) or (markerIds[i] == 112) or (markerIds[i] == 113) or (markerIds[i] == 114) or (markerIds[i] == 121) or (markerIds[i] == 122) or (markerIds[i] == 131) or (markerIds[i] == 132) or (markerIds[i] == 141) or (markerIds[i] == 142) or (markerIds[i] == 201) or (markerIds[i] == 202) or (markerIds[i] == 203) or (markerIds[i] == 204) or (markerIds[i] == 211) or (markerIds[i] == 212) or (markerIds[i] == 213) or (markerIds[i] == 214) or (markerIds[i] == 221) or (markerIds[i] == 222) or (markerIds[i] == 231) or (markerIds[i] == 232) or (markerIds[i] == 241) or (markerIds[i] == 242):
        
        print("Found MPS's -------------")
        msg = 'I have found Tags'
        msg_pub.publish(msg)
        rospy.sleep(2)

        max_x = np.max([last_corner[0],first_corner[0]])
        min_x = np.min([last_corner[0],first_corner[0]])

        max_y = np.max([last_corner[1],first_corner[1]])
        min_y = np.min([last_corner[1],first_corner[1]])
        
        cent = (int(max_x - (max_x-min_x)/2),int(max_y - (max_y-min_y)/2))
        image = cv2.rectangle(aruco_img, first_corner, last_corner, (255, 255, 0), thickness)
        cv2.circle(aruco_img, (cent), 5, (0, 255, 255), -1)
        pos_x = float(arr[cent][0])
        pos_y = float(arr[cent][1])
        pos_z = float(arr[cent][2])

        if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
            aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = pos_x, pos_y, pos_z

        br_ar = tf.TransformBroadcaster()
        br_ar.sendTransform((aruco_pose.point.z, -aruco_pose.point.x, -aruco_pose.point.y), (0.0, 0.0, 0.0, 1.0),rospy.Time.now(), "aruco_rgbd_sensor_link", "camera_link")

  cv2.imshow("Aruco Tags", aruco_img)
  cv2.waitKey(3)

  try:
    aruco_pos_pub.publish(aruco_pose)
  except CvBridgeError as e:
    print(e)

def main(args):
  rospy.init_node('aruco_det', anonymous=True)

  global arr, depth_img_bgr, aruco_pos_pub, depth_points_sub, msg_pub
  print("Image Processing Node - Looking for piece")

  depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  aruco_pos_pub     = rospy.Publisher("/aruco_pos",PointStamped,queue_size=10)
  msg_pub           = rospy.Publisher("/speak", String,queue_size=1)
  msg = String()
  
  depth_img = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)