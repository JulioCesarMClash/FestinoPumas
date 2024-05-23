#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
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
import yaml
from vision_msgs.srv import RecognizeObjects
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from yaml.loader import SafeLoader
from img_proc.srv import *

roslib.load_manifest('img_proc')
bridge = CvBridge()

global Align_Srv

def callback_align_flag(data):
  global begin_flag, Align_Srv
  begin_flag = data

def callback_img(data):
  global img_bgr, Align_Srv
  bridge = CvBridge()
  img_bgr = bridge.imgmsg_to_cv2(data, "bgr8")
  line_detector()
  #cv2.imshow("Aligning",img_bgr)
  cv2.waitKey(1)

def line_detector():
  global img_bgr, slope, Align_Srv
  lines = np.zeros(3)
  average = 1
  detected_lines=[]
  slopes=[]
  edges = cv2.Canny(img_bgr,150,150)
  lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
  cont = 0
  if lines.shape[0] is not None:
      for line in lines:
          cont = cont + 1
          rho, theta = line[0]
          deg_theta = 90 - np.rad2deg(theta)
          if(abs(deg_theta) < 70) or (abs(deg_theta) > 110):
              a = np.cos(theta)
              b = np.sin(theta)
              x0 = a * rho
              y0 = b * rho
              x1 = int(x0 + 1000 * (-b))
              y1 = int(y0 + 1000 * (a))
              x2 = int(x0 - 1000 * (-b))
              y2 = int(y0 - 1000 * (a))
              slope = (y2-y1)/(x2-x1) if (x2-x1)!=0 else 0
              if slope < 0.2 and slope > -0.2:
                  slopes.append(slope)
                  detected_lines.append([x1,y1,x2,y2,slope])
                  cv2.line(img_bgr, (x1, y1), (x2, y2), (0, 0, 255), 4)
  else:
      print("No line detected") 
  if len(slopes) != 0:
      average = sum(slopes)/len(slopes)     
  slope = average

def align(request):
  response = Align_SrvResponse()
  response.success = False
  global begin_flag, slope, img_bgr, pub_vel, Align_Srv
  print("Request",request.is_align_srv_enabled)
  if request.is_align_srv_enabled:  
    error = slope
    Kp = -6.0
    Kp_m = 6.0
    vel = Twist()
    while abs(error) > 0.02:
        if (error < 0) and (error != 1) :
            vel.angular.z = Kp_m*abs(error)
            error = slope
            print("Publishing Vels")
            pub_vel.publish(vel)
        elif error > 0 and (error != 1) :
            vel.angular.z = Kp*abs(error)
            error = slope
            print("Error 3 ", error)
            print("Publishing Vels")
            pub_vel.publish(vel)
    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = 0
    print("Lined up")
    response = Align_SrvResponse()
    response.success = True
    pub_vel.publish(vel)
  else:
      print("Not activated")
      response = Align_SrvResponse()
      response.success = False 
  return response

def main():
  rospy.init_node('Align_srv', anonymous=True)
  rate = rospy.Rate(10.0)
  global begin_flag, pub_vel, Align_Srv, slope
  begin_flag = False

  print("Image Processing Node - Align")

  pub_vel         = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
  sub_img         = rospy.Subscriber("/camera/rgb/image_color", Image, callback_img)
  sub_align      = rospy.Subscriber("/align_flag", Bool, callback_align_flag)
  align_service  = rospy.Service('/vision/align', Align_Srv, align)
  slope = 0

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()
  rospy.spin()