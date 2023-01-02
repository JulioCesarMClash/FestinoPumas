#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('img_proc')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_depth = self.bridge.imgmsg_to_cv2(data, "8UC1")
    except CvBridgeError as e:
      print(e)
    cv_depth = cv_depth*51 #Para escalarlo a 0-255
    #print(cv_depth)
    depth = cv_depth*31.25/10 # Para la distancia en cm?
    print(depth)
    cv2.imshow("Original Image", cv_depth)
    cv2.waitKey(3)

    #try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_depth, "16UC1"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)