#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('img_proc')

import cv2
import numpy as np
import sys
import rospy

from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import *

bridge = CvBridge()


def callback_image(data):
	print('entro al callback')
	try:
		markerImage = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
	parameters = cv2.aruco.DetectorParameters_create()
	markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(markerImage, dictionary, parameters=parameters)
	print(markerIds)

	cv2.imshow("marker12.png", markerImage)
	cv2.waitKey(3)

def main(args):
	rospy.init_node('aruco_tags_det', anonymous=True)
	print("Looking for ARUCO Tag")
	image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,callback_image)
	markerImage = np.zeros((200,200), dtype = np.uint8)
	#markerImage = cv2.aruco.drawMarker(dictionary, 12, 200, markerImage, 1)

	cv2.imwrite("marker12.png", markerImage)

if __name__ == '__main__':
    main(sys.argv)
