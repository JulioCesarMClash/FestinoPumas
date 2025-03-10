#!/usr/bin/env python
from __future__ import print_function
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
import roslib

#Festino dep
from img_proc.srv import *

roslib.load_manifest('img_proc')

def segment_color(img_bgr, img_xyz, hsv_mean):
	img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
	delta = 115
	up_h = hsv_mean[0] + 10
	lw_h = hsv_mean[0] - 10
	up_s = hsv_mean[1] + 30
	lw_s = hsv_mean[1] - 30
	up_v = hsv_mean[2] + delta
	lw_v = hsv_mean[2] - delta
	up = (up_h, up_s, up_v, 0.0)
	lw = (lw_h, lw_s, lw_v, 0.0)
	Fil = cv2.inRange(img_hsv, lw, up)
	return Fil


class FindObjectNode:
	def __init__(self):
		global arr
		rospy.init_node('find_piece_service')
		self.bridge = CvBridge()
  		arr = np.zeros((480, 640))
  		self.find_piece_service = rospy.Service('/vision/find_piece/point_stamped', Find_piece_Srv, self.find_piece)
		self.depth_points_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2, self.callback_depth_points)
		print("Image Processing Srv - Looking for Piece")


	def callback_depth_points(self, data):
		global arr
  		arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)
  		return

	def spin(self):
		rospy.spin()


	def find_piece(self, request):
		global arr, img_bgr, Red_mask
		if request.is_find_piece_enabled:
			rgb_arr = arr['rgb'].copy()
			rgb_arr.dtype = np.uint32
			r, g, b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
			img_bgr = cv2.merge((np.asarray(b, dtype='uint8'), np.asarray(
				g, dtype='uint8'), np.asarray(r, dtype='uint8')))


			################################### Convert in a DataBase ###################################

			# Medias de color en hsv de la pieza en distintas iluminaciones

			####################### BASES #######################

			################# RedBase #################

			mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			Red_mask = cv2.medianBlur(Red_mask3, 9)

			################# RedBase #################

			################# BlackBase #################

			mean_B_l1 = (14.212603109137055, 106.1783788071066, 32.70518718274111, 0.0)
			mean_B_l2 = (2.2857142857142856, 1.6190476190476188, 4.594557823129251, 0.0)
			mean_B_l3 = (116.73317910160016, 39.6176980913823, 87.36148062463852, 0.0)
			mean_B_l4 = (104.055421686747, 87.83603504928807, 4.847535596933188, 0.0)

			Fil_B_l1 = segment_color(img_bgr, arr, mean_B_l1)
			Fil_B_l2 = segment_color(img_bgr, arr, mean_B_l2)
			Fil_B_l3 = segment_color(img_bgr, arr, mean_B_l3)
			Fil_B_l4 = segment_color(img_bgr, arr, mean_B_l4)

			Black_mask1 = cv2.bitwise_or(Fil_B_l1, Fil_B_l2)
			Black_mask2 = cv2.bitwise_or(Black_mask1, Fil_B_l3)
			Black_mask3 = cv2.bitwise_or(Black_mask2, Fil_B_l4)
			Black_mask = cv2.medianBlur(Black_mask3, 9)

			################# BlackBase #################

			################# TransBase #################

			mean_T_l1 = (17.590204678362575, 22.059722222222224, 176.96319444444444, 0.0)
			mean_T_l2 = (13.521739130434781, 58.07959866220735, 160.16321070234113, 0.0)
			mean_T_l3 = (16.946801346801347, 54.16026936026936, 176.8929292929293, 0.0)
			mean_T_l4 = (17.399466933200067, 63.67999333666501, 172.57871064467767, 0.0)

			Fil_T_l1 = segment_color(img_bgr, arr, mean_T_l1)
			Fil_T_l2 = segment_color(img_bgr, arr, mean_T_l2)
			Fil_T_l3 = segment_color(img_bgr, arr, mean_T_l3)
			Fil_T_l4 = segment_color(img_bgr, arr, mean_T_l4)

			Trans_mask1 = cv2.bitwise_or(Fil_T_l1, Fil_T_l2)
			Trans_mask2 = cv2.bitwise_or(Trans_mask1, Fil_T_l3)
			Trans_mask3 = cv2.bitwise_or(Trans_mask2, Fil_T_l4)
			Trans_mask = cv2.medianBlur(Trans_mask3, 9)

			################# TransBase #################

			################# SilverBase #################

			mean_T_l1 = (10.387058823529411, 25.79891402714932, 180.71447963800904, 0.0)
			mean_T_l2 = (15.917393199651265, 79.19354838709678, 63.51046207497821, 0.0)
			mean_T_l3 = (14.4558066486345, 29.16612861006893, 197.8072594014484, 0.0)
			mean_T_l4 = (16.316959064327484, 47.644152046783624, 82.54736842105264, 0.0)

			Fil_T_l1 = segment_color(img_bgr, arr, mean_T_l1)
			Fil_T_l2 = segment_color(img_bgr, arr, mean_T_l2)
			Fil_T_l3 = segment_color(img_bgr, arr, mean_T_l3)
			Fil_T_l4 = segment_color(img_bgr, arr, mean_T_l4)

			Trans_mask1 = cv2.bitwise_or(Fil_T_l1, Fil_T_l2)
			Trans_mask2 = cv2.bitwise_or(Trans_mask1, Fil_T_l3)
			Trans_mask3 = cv2.bitwise_or(Trans_mask2, Fil_T_l4)
			Trans_mask = cv2.medianBlur(Trans_mask3, 9)

			################# SilverBase #################

			####################### RINGS #######################

			################# YellowRing #################

			# mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			# mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			# mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			# mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			# Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			# Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			# Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			# Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			# Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			# Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			# Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			# Red_mask = cv2.medianBlur(Red_mask3, 9)

			# ################# YellowRing #################

			# ################# GreenRing #################

			# mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			# mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			# mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			# mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			# Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			# Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			# Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			# Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			# Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			# Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			# Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			# Red_mask = cv2.medianBlur(Red_mask3, 9)

			# ################# GreenRing #################

			# ################# BlueRing #################

			# mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			# mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			# mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			# mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			# Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			# Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			# Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			# Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			# Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			# Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			# Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			# Red_mask = cv2.medianBlur(Red_mask3, 9)

			# ################# BlueRing #################

			# ################# OrangeRing #################

			# mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			# mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			# mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			# mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			# Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			# Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			# Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			# Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			# Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			# Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			# Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			# Red_mask = cv2.medianBlur(Red_mask3, 9)

			# ################# OrangeRing #################

			# ####################### CAPS #######################

			# ################# WhiteRing #################

			# mean_R_l1 = (171.79478086924894, 219.36252045826512, 248.39407164939078, 0.0)
			# mean_R_l2 = (168.52717391304347, 173.29021739130434, 107.51413043478261, 0.0)
			# mean_R_l3 = (168.7984375, 212.20364583333333, 110.8015625, 0.0)
			# mean_R_l4 = (174.05156250000002, 253.4765625, 165.7984375, 0.0)

			# Fil_R_l1 = segment_color(img_bgr, arr, mean_R_l1)
			# Fil_R_l2 = segment_color(img_bgr, arr, mean_R_l2)
			# Fil_R_l3 = segment_color(img_bgr, arr, mean_R_l3)
			# Fil_R_l4 = segment_color(img_bgr, arr, mean_R_l4)

			# Red_mask1 = cv2.bitwise_or(Fil_R_l1, Fil_R_l2)
			# Red_mask2 = cv2.bitwise_or(Red_mask1, Fil_R_l3)
			# Red_mask3 = cv2.bitwise_or(Red_mask2, Fil_R_l4)
			# Red_mask = cv2.medianBlur(Red_mask3, 9)

			# ################# WhiteRing #################

			# ################################### Convert in a DataBase ###################################

			######## Filling msg for tapita_pose publisher ########
			piece_pose = PointStamped()
			piece_pose.header.stamp = rospy.Time.now()
			piece_pose.header.frame_id = "camera_link"
			piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = 0, 0, 0

			aruco_pose = piece_pose

			######## Filling msg for tapita_static_pose publisher ########

			static_transformStamped = geometry_msgs.msg.TransformStamped()

			static_transformStamped.header.stamp = rospy.Time.now()
			static_transformStamped.header.frame_id = "map"
			static_transformStamped.child_frame_id = "piece_static_link"

			static_transformStamped.transform.rotation.x = 0.0
			static_transformStamped.transform.rotation.y = 0.0
			static_transformStamped.transform.rotation.z = 0.0
			static_transformStamped.transform.rotation.w = 1.0

			######## Looking for Piece ########

			Masked_red = np.zeros((480, 640))
			img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
			loc = cv2.findNonZero(Trans_mask)

			# loc[i,0,0] : Points_i
			# loc[i,0,1] : Points_j
			for i in range(loc.shape[0]):
				pos_x = float(arr[loc[i, 0, 1], loc[i, 0, 0]][0])
				pos_y = float(arr[loc[i, 0, 1], loc[i, 0, 0]][1])
				pos_z = float(arr[loc[i, 0, 1], loc[i, 0, 0]][2])

				static_transformStamped.transform.translation.x = pos_z
				static_transformStamped.transform.translation.y = -pos_x
				static_transformStamped.transform.translation.z = -pos_y
				# -Sacar la media de los puntos


				if not (math.isnan(pos_x) or math.isnan(pos_y) or math.isnan(pos_z)):
					piece_pose.point.x, piece_pose.point.y, piece_pose.point.z = pos_x, pos_y, pos_z
					br = tf.TransformBroadcaster()
					static_br = tf2_ros.StaticTransformBroadcaster()
					rate = rospy.Rate(0.1)
					br.sendTransform((piece_pose.point.z, -piece_pose.point.x, -piece_pose.point.y),
			                   (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "piece_link", "camera_link")
					static_br.sendTransform(static_transformStamped)

					M = cv2.moments(Red_mask)

					try:
						cX = int(M["m10"] / M["m00"])
						cY = int(M["m01"] / M["m00"])
						cv2.circle(img_bgr, (cX, cY), 5, (255, 255, 255), -1)
						cv2.putText(img_bgr, "centroid_Red", (cX - 25, cY - 25),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
						Masked_red = cv2.bitwise_and(img_bgr, img_bgr, mask=Red_mask)
						print('Tapita Found')

					except ZeroDivisionError as e:
						print("Object not found")

					mean_B_l1 = (73.37954475229168, 93.521268925739, 0.0, 0.0)
					Fil_B_l1 = segment_color(img_bgr, arr, mean_B_l1)

				else:
  					print('Unable to find Tapita')
			
			response = Find_piece_SrvResponse()
			response.success = True
			response.point_stamped = piece_pose

			#cv2.imshow("Aruco Tags", img_bgr)
			#cv2.imshow("Red Piece Mask", Red_mask)
			#cv2.waitKey(3)

			return response
		
		else:

			response = Find_piece_SrvResponse()
			response.success = False
			response.point_stamped.point.x = 0.0
			response.point_stamped.point.y = 0.0
			response.point_stamped.point.z = 0.0

			return response


if __name__ == '__main__':
    find_piece_node = FindObjectNode()
    find_piece_node.spin()





