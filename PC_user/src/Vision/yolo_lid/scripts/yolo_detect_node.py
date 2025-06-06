#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from yolo_detect.msg import StringArray
import logging
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Point
import message_filters
import numpy as np

logging.getLogger('ultralytics').setLevel(logging.WARNING)

def load_model():
    model_path = rospy.get_param('~model_path', '/models/best.pt')
    model = YOLO(model_path)
    rospy.loginfo(f"Loaded YOLOv8 model from {model_path}")
    return model

class YoloCategoryNode:
    def __init__(self):
        self.model = load_model()
        self.bridge = CvBridge()

        # Suscribirse a los mensajes de imagen y cámara
        image_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Sincronización de los mensajes de imagen y profundidad
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self.image_callback)

        self.centroid_pub = rospy.Publisher("/vision/lid_centroid", Point, queue_size=10)

        # Valores intrínsecos inicializados a `None`, su valor se establecerá mediante `camera_info_callback`
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        # rospy.loginfo("Calibración actualizada")

    def image_callback(self, img_msg, depth_msg):
        if self.fx is None or self.fy is None:
            rospy.logwarn("Camera intrinsics not yet initialized.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return

        results = self.model(cv_image, conf=0.7)[0]

        for result in results:
            annotated_frame = result.plot()
            cv2.imshow("YOLOv8 Lid Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Closed by user")

        all_boxes = results.boxes.data.cpu().numpy()
        if len(all_boxes) > 0:
            areas = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])
            selected_idx = areas.argmax()
            selected_box = all_boxes[selected_idx]

            centroid_x = int((selected_box[0] + selected_box[2]) / 2)
            centroid_y = int((selected_box[1] + selected_box[3]) / 2)

            depth = depth_image[centroid_y, centroid_x]
            if depth == 0:
                rospy.logwarn("Depth data invalid at centroid. Skipping.")
                # return

            z = depth / 1000.0  # Convertir a metros si el mapa de profundidad está en milímetros
            x = (centroid_x - self.cx) * z / self.fx
            y = (centroid_y - self.cy) * z / self.fy

            centroid_msg = Point()
            centroid_msg.x = x
            centroid_msg.y = y
            centroid_msg.z = z #int(selected_box[-1]) 
            self.centroid_pub.publish(centroid_msg)

if __name__ == '__main__':
    rospy.init_node('yolo_lid_node')
    node = YoloCategoryNode()
    rospy.loginfo("YOLO lid node started")
    rospy.spin()
