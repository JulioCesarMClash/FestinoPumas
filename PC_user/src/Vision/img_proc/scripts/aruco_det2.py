#!/usr/bin/env python
from __future__ import print_function

import time

#Para desempacar la matriz de la cámara y los coeficientes de distor
import pickle

from scipy.spatial.transform import Rotation
import numpy as np

import roslib
import tf.transformations
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

# def aruco_mps(aruco_id):
#   ######## CYAN STATIONS ########
#   #CapStations
#   mps_name_arr = 0
#   if(aruco_id == 101):
#     mps_name_arr = ['C','CS1','O']
#   elif(aruco_id == 102):
#     mps_name_arr = ['C','CS1','I']
#   elif(aruco_id == 103):
#     mps_name_arr = ['C','CS2','O']
#   elif(aruco_id == 104):
#     mps_name_arr = ['C','CS2','I']

#   #RingStations
#   elif(aruco_id == 111):
#     mps_name_arr = ['C','RS1','O']
#   elif(aruco_id == 112):
#     mps_name_arr = ['C','RS1','I']
#   elif(aruco_id == 113):
#     mps_name_arr = ['C','RS2','O']
#   elif(aruco_id == 114):
#     mps_name_arr = ['C','RS2','I']

#   #BaseStations
#   elif(aruco_id == 121):
#     mps_name_arr = ['C','BS','O']
#   elif(aruco_id == 122):
#     mps_name_arr = ['C','BS','I']

#   #DeliveryStations
#   elif(aruco_id == 131):
#     mps_name_arr = ['C','DS','O']
#   elif(aruco_id == 132):
#     mps_name_arr = ['C','DS','I']
  
#   #StorageStations
#   elif(aruco_id == 141):
#     mps_name_arr = ['C','SS','O']
#   elif(aruco_id == 142):
#     mps_name_arr = ['C','SS','I']

#   ######## MAGENTA STATIONS ########
#   #CapStations
#   elif(aruco_id == 201):
#     mps_name_arr = ['M','CS1','O']
#   elif(aruco_id == 202):
#     mps_name_arr = ['M','CS1','I']
#   elif(aruco_id == 203):
#     mps_name_arr = ['M','CS2','O']
#   elif(aruco_id == 204):
#     mps_name_arr = ['M','CS2','I']

#   #RingStations
#   elif(aruco_id == 211):
#     mps_name_arr = ['M','RS1','O']
#   elif(aruco_id == 212):
#     mps_name_arr = ['M','RS1','I']
#   elif(aruco_id == 213):
#     mps_name_arr = ['M','RS2','O']
#   elif(aruco_id == 214):
#     mps_name_arr = ['M','RS2','I']

#   #BaseStations
#   elif(aruco_id == 221):
#     mps_name_arr = ['M','BS','O']
#   elif(aruco_id == 222):
#     mps_name_arr = ['M','BS','I']

#   #DeliveryStations
#   elif(aruco_id == 231):
#     mps_name_arr = ['M','DS','O']
#   elif(aruco_id == 232):
#     mps_name_arr = ['M','DS','I']
  
#   #StorageStations
#   elif(aruco_id == 241):
#     mps_name_arr = ['M','SS','O']
#   elif(aruco_id == 242):
#     mps_name_arr = ['M','SS','I']
#   mps_name = mps_name_arr[0]+'-'+mps_name_arr[1]+'-'+mps_name_arr[2]
#   return mps_name_arr, mps_name

def aruco_mps(aruco_id):
  ######## CYAN STATIONS ########
  #CapStations
  mps_name_arr = 0
  if(aruco_id == 101):
    mps_name_arr = ['C','CS1']
  elif(aruco_id == 102):
    mps_name_arr = ['C','CS1']
  elif(aruco_id == 103):
    mps_name_arr = ['C','CS2']
  elif(aruco_id == 104):
    mps_name_arr = ['C','CS2']

  #RingStations
  elif(aruco_id == 111):
    mps_name_arr = ['C','RS1']
  elif(aruco_id == 112):
    mps_name_arr = ['C','RS1']
  elif(aruco_id == 113):
    mps_name_arr = ['C','RS2']
  elif(aruco_id == 114):
    mps_name_arr = ['C','RS2']

  #BaseStations
  elif(aruco_id == 121):
    mps_name_arr = ['C','BS']
  elif(aruco_id == 122):
    mps_name_arr = ['C','BS']

  #DeliveryStations
  elif(aruco_id == 131):
    mps_name_arr = ['C','DS']
  elif(aruco_id == 132):
    mps_name_arr = ['C','DS']
  
  #StorageStations
  elif(aruco_id == 141):
    mps_name_arr = ['C','SS']
  elif(aruco_id == 142):
    mps_name_arr = ['C','SS']

  ######## MAGENTA STATIONS ########
  #CapStations
  elif(aruco_id == 201):
    mps_name_arr = ['M','CS1']
  elif(aruco_id == 202):
    mps_name_arr = ['M','CS1']
  elif(aruco_id == 203):
    mps_name_arr = ['M','CS2']
  elif(aruco_id == 204):
    mps_name_arr = ['M','CS2']

  #RingStations
  elif(aruco_id == 211):
    mps_name_arr = ['M','RS1']
  elif(aruco_id == 212):
    mps_name_arr = ['M','RS1']
  elif(aruco_id == 213):
    mps_name_arr = ['M','RS2']
  elif(aruco_id == 214):
    mps_name_arr = ['M','RS2']

  #BaseStations
  elif(aruco_id == 221):
    mps_name_arr = ['M','BS']
  elif(aruco_id == 222):
    mps_name_arr = ['M','BS']

  #DeliveryStations
  elif(aruco_id == 231):
    mps_name_arr = ['M','DS']
  elif(aruco_id == 232):
    mps_name_arr = ['M','DS']
  
  #StorageStations
  elif(aruco_id == 241):
    mps_name_arr = ['M','SS']
  elif(aruco_id == 242):
    mps_name_arr = ['M','SS']
  mps_name = mps_name_arr[0]+'-'+mps_name_arr[1]#+'-'+mps_name_arr[2]
  return mps_name_arr, mps_name

def callback_depth_points(data):
  global rate, arr
  listener = tf.TransformListener()
  arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
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
  corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(aruco_img, dictionary, parameters=parameters)

  color = (255, 0, 0)
  thickness = 2
  aruco_det_flag = False
  mps_name = "Not Identified"
  fracaso = "fracaso"

  with open('/home/robocup20/FestinoPumas/PC_user/src/Vision/img_proc/scripts/cameraMatrix.pkl', 'rb') as f:
        mtx = pickle.load(f)

    
  with open('/home/robocup20/FestinoPumas/PC_user/src/Vision/img_proc/scripts/dist.pkl', 'rb') as f:
        dst = pickle.load(f)

  try:
    if(markerIds.shape[0] >= 1):
      
      for i in range (markerIds.shape[0]):

        corneru = corners[0]

        start_point = (corneru[(0,0,0)],corneru[(0,0,1)])

        # End coordinate, here (250, 250) 
        # represents the bottom right corner of image 
        end_point = (corneru[(0,3,0)],corneru[(0,3,1)])

        y2 = corneru[(0,3,1)]
        y1 = corneru[(0,0,1)]
        x2 = corneru[(0,3,0)]
        x1 = corneru[(0,0,0)]
        
        # Green color in BGR 
        color = (0, 255, 0) 
        
        # Line thickness of 9 px 
        thickness = 9
        
        # Using cv2.line() method 
        # Draw a diagonal green line with thickness of 9 px 
        image = cv2.line(aruco_img, start_point, end_point, color, thickness) 

        slope = (y2-y1)/(x2-x1) if (x2-x1)!=0 else 0
        print("la pendiente es: ", slope)

        if(slope < 0.03  and slope > -0.03):
          print("alineado!!")
        else:
          print("ño")

        slope_pub.publish(slope)

        #corners = markerCorners[i]

        # flatten the ArUco IDs list
        #ids = ids.flatten()
        # loop over the detected ArUCo corners
        #for (markerCorner, markerID) in zip(corners, ids):
        #(markerCorner, markerID)=(corners, ids)
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
        #corners = corners.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0][0],corners[0][0][1],corners[0][0][2],corners[0][0][3]
            # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
        cv2.line(aruco_img, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(aruco_img, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(aruco_img, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(aruco_img, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(aruco_img, (cX, cY), 4, (0, 0, 255), -1)
        
        # if topLeft[1]!=topRight[1] or topLeft[0]!=bottomLeft[0]:
        #     rot1=np.degrees(np.arctan((topLeft[0]-bottomLeft[0])/(bottomLeft[1]-topLeft[1])))
        #     rot2=np.degrees(np.arctan((topRight[1]-topLeft[1])/(topRight[0]-topLeft[0])))
        #     rot=(np.round(rot1,3)+np.round(rot2,3))/2
        #     print(rot1,rot2,rot)
        # else:
        #     rot=0

        # # draw the ArUco marker ID on the image
        # rotS=",rotation:"+str(np.round(rot,3))
        # cv2.putText(aruco_img, ("position: "+str(cX) +","+str(cY)),
        # (100, topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 80), 2)
        # cv2.putText(aruco_img, rotS,
        # (400, topLeft[1] -15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 80), 2)
        # #print("[INFO] ArUco marker ID: {}".format(ids))

        
        # d=np.round((math.dist(topLeft,bottomRight)+math.dist(topRight,bottomLeft))/2,3)
        # # Get the rotation and translation vectors
        aruco_marker_side_length = 0.123 
        #12.3 cm o 0.123 m
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners,aruco_marker_side_length,mtx,dst)
            
        # Print the pose for the ArUco marker
        # The pose of the marker is with respect to the camera lens frame.
        # Imagine you are looking through the camera viewfinder, 
        # the camera lens frame's:
        # x-axis points to the right
        # y-axis points straight down towards your toes
        # z-axis points straight ahead away from your eye, out of the camera
        #for i, marker_id in enumerate(marker_ids):
            
        #Store the translation (i.e. position) information
        transform_translation_x = tvecs[0][0][0]
        transform_translation_y = tvecs[0][0][1]
        transform_translation_z = tvecs[0][0][2]

        #Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0]))[0]
        try: 
          r = Rotation.from_dcm(rotation_matrix[0:3, 0:3])
        except Exception as e: 
            print(e)
            print('No se pudo por alguna razón :(')

        cv2.drawFrameAxes(aruco_img, mtx, dst, rvecs, tvecs, 0.123 * 1.5, 2)
        #cv2.solvePnP(obj_points, corners, mtx, dst, rvecs, tvecs)
        quat = r.as_quat()   
        
        #Quaternion format     
        transform_rotation_x = quat[0] 
        transform_rotation_y = quat[1] 
        transform_rotation_z = quat[2] 
        transform_rotation_w = quat[3] 

        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = tf.transformations.euler_from_quaternion([transform_rotation_x,transform_rotation_y,transform_rotation_z,transform_rotation_w])
                
        roll_x = math.degrees(roll_x)
        pitch_y = math.degrees(pitch_y)
        yaw_z = math.degrees(yaw_z)
        #print(roll_x, pitch_y, yaw_z)

        time.sleep(0.1)

        # print("Siii")
        # cv2.drawFrameAxes(aruco_img, mtx, dst, rvecs, tvecs, 0.123 * 1.5, 2)
        # print("Siiix2")

  except AttributeError:
    print('No Tag') 

  cv2.imshow("Aruco Tags", aruco_img)
  cv2.waitKey(3)

  try:
    aruco_flag_pub.publish(aruco_det_flag)
  except CvBridgeError as e:
    print(e)


def main(args):
  rospy.init_node('aruco_det', anonymous=True)

  global rate, arr, depth_img_bgr, aruco_pos_pub, depth_points_sub, aruco_flag_pub, mps_data_pub, mps_name_pub, slope_pub
  print("Image Processing Node - Looking for piece")
  rate = rospy.Rate(10)
  depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,callback_depth_points)
  aruco_pos_pub     = rospy.Publisher("/mps_pos",PointStamped,queue_size=10)
  aruco_flag_pub    = rospy.Publisher("/aruco_det",Bool,queue_size=10)
  mps_name_pub    = rospy.Publisher("/mps_name",String,queue_size=10)
  mps_data_pub    = rospy.Publisher("/mps_data",String,queue_size=10)
  slope_pub    = rospy.Publisher("/slope_data",Float32,queue_size=10)

  
  depth_img_bgr = np.zeros((480, 640))
  arr = np.zeros((480, 640))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)