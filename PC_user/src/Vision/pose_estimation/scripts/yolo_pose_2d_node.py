#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

from pose_estimation.msg import PersonPose2D, Keypoint2D

KEYPOINT_NAMES = [
    "nose", "eye_left", "eye_right", "ear_left", "ear_right",
    "shoulder_left", "shoulder_right", "elbow_left", "elbow_right",
    "wrist_left", "wrist_right", "hip_left", "hip_right",
    "knee_left", "knee_right", "ankle_left", "ankle_right"
]

class YoloPoseNode:
    def __init__(self):
        rospy.init_node("yolo_pose_2d_node")
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt")  
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/vision/pose_2d", PersonPose2D, queue_size=10)

        # Definir conexiones del esqueleto
        self.skeleton = [
            [15, 13], [13, 11], [16, 14], [14, 12], [11, 12],
            [5, 11], [6, 12], [5, 6], [5, 7], [6, 8],
            [7, 9], [8, 10], [0, 1], [0, 2], [1, 3], [2, 4]
        ]

        rospy.loginfo("YOLO Node 2D - Modo: Persona más cercana")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Error procesando imagen: {e}")

    def create_person_message(self, person_id, keypoints):
        person_msg = PersonPose2D()
        person_msg.id = person_id
        person_msg.keypoints = []

        for i, (x, y, conf) in enumerate(keypoints):
            kp_msg = Keypoint2D()
            kp_msg.name = KEYPOINT_NAMES[i]
            kp_msg.x = float(x)
            kp_msg.y = float(y)
            kp_msg.confidence = float(conf)
            person_msg.keypoints.append(kp_msg)

        return person_msg

    def process_image(self, cv_image):
        if not rospy.get_param('/pose_2d_enabled', True):
            return

        # Procesar con YOLO
        results = self.model(cv_image)[0]
        
        # Extraer bounding boxes y keypoints
        boxes = results.boxes.xyxy.cpu().numpy()
        keypoints = results.keypoints.data.cpu().numpy()

        if len(boxes) == 0:
            return  # No hay detecciones

        # Calcular áreas y seleccionar el bounding box más grande
        areas = [(box[2]-box[0])*(box[3]-box[1]) for box in boxes]
        max_idx = np.argmax(areas)
        selected_box = boxes[max_idx]
        selected_kpts = keypoints[max_idx]

        # Publicar solo la persona seleccionada
        self.pub.publish(self.create_person_message(0, selected_kpts))

        # Visualización mejorada
        annotated_image = cv_image.copy()
        
        # Dibujar bounding box
        x1, y1, x2, y2 = map(int, selected_box)
        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Dibujar keypoints y esqueleto
        for kp in selected_kpts:
            x, y, conf = kp
            if conf > 0.5:
                cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 0, 255), -1)

        for connection in self.skeleton:
            start_idx, end_idx = connection
            start = selected_kpts[start_idx]
            end = selected_kpts[end_idx]
            
            if start[2] > 0.5 and end[2] > 0.5:
                start_pt = (int(start[0]), int(start[1]))
                end_pt = (int(end[0]), int(end[1]))
                cv2.line(annotated_image, start_pt, end_pt, (255, 0, 0), 2)

        cv2.imshow("Persona Principal - YOLOv8 Pose", annotated_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        YoloPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass