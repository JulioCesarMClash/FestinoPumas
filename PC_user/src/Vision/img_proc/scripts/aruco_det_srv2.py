#!/usr/bin/env python
from __future__ import print_function

import time

#Para desempacar la matriz de la camara y los coeficientes de distor
import pickle

#from scipy.spatial.transform import Rotation

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
from geometry_msgs.msg import Twist


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

    #Este suscriptor recibe la nube de puntos del kinect, mediante el topico "/camera/depth_registered/points"
    self.depth_points_sub  = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.callback_depth_points)

    self.find_tag_service = rospy.Service('/vision/find_tag/point_stamped', Find_tag_Srv, self.find_tag)

  #Este callback es el que recibe la nube de puntos y se guarda el frame en la variable arr
  def callback_depth_points(self, data):
    global arr
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    return

  def spin(self):
    rospy.spin()
    
  def find_tag(self, request):
    global arr, depth_img_bgr, aruco_pos_pub, depth_points_sub, mps_name_pub
    slope = 0.04
    vel = Twist()
    name_list = []
    aruco_list = PointStamped()
    aruco_list = []
    #Bandera para que gire al otro lado cuando busca Aruco
    next_turn = False
    #Bandera que indica que se giro para un lado al buscar el Aruco
    no_find_1 = False
    #Bandera que indica que se giro para el otro lado a buscar el Aruco
    no_find_2 = False 
    #Bandera que indica que ya se encontro el Aruco
    already_tag = False
    #Contador del numero de giros de busqueda 
    cont_giro = 0
    go_back = False
    #Cuando se haga un request a este servicio se debe de poner is_find_tag_enabled=true
    #Cuando se cumpla eso ya se ejecutara lo que esta adentro del if
    #Esta es la primera parte para alinearse en angulo
    if request.is_find_tag_enabled:
      while(slope > 0.01 or slope < -0.01):
        tfBuffer = tf2_ros.Buffer()
        depth_img_bgr = np.zeros((480, 640))
        mps_name = [0,0]
        #A partir de la nube de puntos se obtiene el RGB
        rgb_arr = arr['rgb'].copy()
        rgb_arr.dtype = np.uint32
        r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
        #Se hace un Merge de los 3 canales para obtener la imagen final que se analizara, que es aruco_img
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

        try:
            if(markerIds.shape[0] >= 1):
      
                for i in range (markerIds.shape[0]):
                    corneru = corners[0]
                    first_corner = (corneru[(0,0,0)],corneru[(0,0,1)])
                    last_corner = (corneru[(0,2,0)],corneru[(0,2,1)])
                    known_markers = ([101,102,103,104,111,112,113,114,121,122,131,132,141,142,201,202,203,204,211,212,213,214,221,222,231,232,241,242])

                    if markerIds[i] in known_markers:
                      aruco_det_flag = True
                      max_x = np.max([last_corner[0],first_corner[0]])
                      min_x = np.min([last_corner[0],first_corner[0]])

                      max_y = np.max([last_corner[1],first_corner[1]])
                      min_y = np.min([last_corner[1],first_corner[1]])
                      
                      print("veo un aruco y el tamano de imagen es: ", aruco_img.shape)
                      cent_i = int(max_x - (max_x-min_x)/2)
                      if cent_i > 479:
                        cent_i = 479
                      cent_j = int(max_y - (max_y-min_y)/2)
                      cent = (cent_i,cent_j)
                      #print("el centro es: ",cent)

                      pos_x = float(arr[cent][0])
                      pos_y = float(arr[cent][1])
                      pos_z = float(arr[cent][2])

                      if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
                        #aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = pos_z, -pos_y, -pos_x
                        aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z = pos_z, -pos_y+0.21, -pos_x-0.25
                        print(aruco_pose.point.x, aruco_pose.point.y, aruco_pose.point.z, '\n')

                      #Para el C0
                      #if(aruco_pose.point.x  < 4):

                      #if para que solo detecte RS (grasping challenge)
                      if(markerIds[i] == 111 or markerIds[i] == 112 or markerIds[i] == 113 or markerIds[i] == 114):
                        
                      #if para que solo detecte CS (grasping challenge)
                      #if(markerIds[i] == 101 or markerIds[i] == 102 or markerIds[i] == 103 or markerIds[i] == 104):

                        start_point = (corneru[(0,0,0)],corneru[(0,0,1)])

                        # End coordinate, here (250, 250) 
                        # represents the bottom right corner of image 

                        #Esquina para detectar recta superior
                        #end_point = (corneru[(0,3,0)],corneru[(0,3,1)])
                        
                        #Esquina para detectar recta lateral (con esta orientacion estan los Arucos en las maquinas)
                        end_point = (corneru[(0,1,0)],corneru[(0,1,1)])

                        y2 = corneru[(0,1,1)]
                        y1 = corneru[(0,0,1)]
                        x2 = corneru[(0,1,0)]
                        x1 = corneru[(0,0,0)]
                        
                        # Green color in BGR 
                        color = (0, 255, 0) 
                        
                        # Line thickness of 9 px 
                        thickness = 9
                        
                        # Using cv2.line() method 
                        # Draw a diagonal green line with thickness of 9 px 
                        image = cv2.line(aruco_img, start_point, end_point, color, thickness) 

                        #Se obtiene la pendiente de la recta
                        slope = (y2-y1)/(x2-x1) if (x2-x1)!=0 else 0

                        print("veo un aruco dentro del rango y la pendiente es: ", slope)

                        
                        Kp = -3.0
                        Kp_m = 3.0

                        vel.linear.y = 0
                        if(slope > 0.01):
                            print("giro giro")
                            vel.angular.z = Kp*abs(slope)
                            #Se publica al cmd_vel el giro angular que se requiera
                            #Meti el publish en los if porque al estar afuera hace un giro extra 
                            pub_vel.publish(vel)
                        elif (slope < -0.01):
                            print("giro giro")
                            vel.angular.z = Kp_m*abs(slope)
                            #Se publica al cmd_vel el giro angular que se requiera
                            #Meti el publish en los if porque al estar afuera hace un giro extra 
                            pub_vel.publish(vel)

                        rospy.sleep(3)
                        #Si al principio no lo encontro entonces giro
                        #Estos ifs son para que se mueva hacia el lado que giro para que al querer alinearse no lo pierda de nuevo
                        #Primero se tiene que hacer esto y despues se tiene que sacar la pendiente
                        if not already_tag:
                          vel.angular.z = 0
                          if no_find_2:
                            vel.linear.y = -1
                            pub_vel.publish(vel)
                            print("Me muevo para un lado hmm")
                          elif no_find_1 and not no_find_2:
                            vel.linear.y = 1
                            pub_vel.publish(vel)
                            print("Me muevo para el otro lado")
                            
                        already_tag = True
                        rospy.sleep(2)
                      else:
                        already_tag = False 
                        if next_turn:
                          #Se gira despues para este lado (sentido horario)
                          #Si aun no regresa a la posicion original que gire 3 veces para regresar a 
                          #la posicion original mas un giro extra
                          #if not go_back:
                          vel.angular.z = -0.7854
                          print("Giro para el otro dentro del try aruco fuera rango")
                          #go_back = True
                          #Ya que dio el primer giro da el segundo
                          #else:
                          no_find_2 = True
                        else:
                          #Primero se gira hacia este lado (sentido antihorario)
                          no_find_1 = True
                          print("Giro para un lado dentro del try aruco fuera rango")
                          vel.angular.z = 0.7854
                          #Aumenta en 1 el numero de giros
                          cont_giro = cont_giro + 1
                          #Cuando ya se hayan dado dos giros hacia este lado ya se empezara a girar al otro
                          if(cont_giro == 2):
                              next_turn = True
                        pub_vel.publish(vel)
                        print('No Tag')
                        rospy.sleep(4)

        #Esta excepcion es cuando no encuentra ningun Aruco
        except AttributeError:
            already_tag = False 
            if next_turn:
               #Se gira despues para este lado (sentido horario)
               #Si aun no regresa a la posicion original que gire 3 veces para regresar a 
               #la posicion original mas un giro extra
               #if not go_back:
               vel.angular.z = -0.7854
               print("Giro para el otro")
               #go_back = True
               #Ya que dio el primer giro da el segundo
               #else:
               no_find_2 = True
            else:
               #Primero se gira hacia este lado (sentido antihorario)
               no_find_1 = True
               print("Giro para un lado")
               vel.angular.z = 0.7854
               #Aumenta en 1 el numero de giros
               cont_giro = cont_giro + 1
               #Cuando ya se hayan dado dos giros hacia este lado ya se empezara a girar al otro
               if(cont_giro == 2):
                  next_turn = True

            pub_vel.publish(vel)
            print('No Tag')
            rospy.sleep(4)

      #Cuando termina el while y ya se alineo en angulo manda el success
      print(name_list)
      print(aruco_list)
      response = Find_tag_SrvResponse()
      response.success = True
      response.mps_name = name_list
      response.point_stamped = aruco_list

      return response
    #Este if es para cuando se tiene que alinear en Y
    elif request.is_aling_enabled:
      print("Ya entro a la segunda parte del alineado")
      umbral = 20
      diff = 35
      while(diff > umbral or diff < -umbral):
        tfBuffer = tf2_ros.Buffer()
        depth_img_bgr = np.zeros((480, 640))
        mps_name = [0,0]
        #A partir de la nube de puntos se obtiene el RGB
        rgb_arr = arr['rgb'].copy()
        rgb_arr.dtype = np.uint32
        r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
        #Se hace un Merge de los 3 canales para obtener la imagen final que se analizara, que es aruco_img
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

        try:
            if(markerIds.shape[0] >= 1):
      
                for i in range (markerIds.shape[0]):
                    corneru = corners[0]
                    first_corner = (corneru[(0,0,0)],corneru[(0,0,1)])
                    last_corner = (corneru[(0,2,0)],corneru[(0,2,1)])
                    known_markers = ([101,102,103,104,111,112,113,114,121,122,131,132,141,142,201,202,203,204,211,212,213,214,221,222,231,232,241,242])

                    if markerIds[i] in known_markers:
                      aruco_det_flag = True
                      max_x = np.max([last_corner[0],first_corner[0]])
                      min_x = np.min([last_corner[0],first_corner[0]])

                      max_y = np.max([last_corner[1],first_corner[1]])
                      min_y = np.min([last_corner[1],first_corner[1]])
                      
                      print("tamano de imagen es: ", aruco_img.shape)
                      cent = (int(max_x - (max_x-min_x)/2),int(max_y - (max_y-min_y)/2))
                      print("El centro del aruco es: ", cent)
                      cent_img = aruco_img.shape[1]/2+5
                      diff = cent_img - cent[0]
                      print("La mitad de la imagen es: ", cent_img)
                      print("La diff es: ", diff)
                      
                      Kp = -0.02
                      Kp_m = 0.02

                      if diff > umbral:
                         vel.linear.y = Kp_m*abs(diff)
                         #Se publica al cmd_vel el movimiento en y del robot
                         print("Entro al if de 20")
                         pub_vel.publish(vel)
                      elif diff < -umbral:
                         print("Entro al if de -20")
                         vel.linear.y = Kp*abs(diff)
                         pub_vel.publish(vel)
                    #Delay para que le de tiempo al robot de moverse
                    rospy.sleep(3)


        except AttributeError:
          
            print('No Tag')

      #vel.linear.x = 2*pos_z	
      #vel.linear.y = 0
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
  global pub_vel
  pub_vel  = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
  #pubLateral = rospy.Publisher("/move_lateral",Float32,queue_size=1)

  find_tag_node.spin()
