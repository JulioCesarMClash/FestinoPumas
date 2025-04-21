#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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
        self.pub = rospy.Publisher("pose_2d", PersonPose2D, queue_size=10)

        rospy.loginfo("YOLO Node 2D --- Soft by Joshua M")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Error to convert image: {e}")
            return
        
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
        results = self.model(cv_image)[0]
        
        for result in results:
            annotated_frame = result.plot()
            cv2.imshow("YOLOv8 Pose Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Closed by user")

        for person_id, keypoints in enumerate(results.keypoints.data.cpu().numpy()):
            person_msg = self.create_person_message(person_id, keypoints)
            self.pub.publish(person_msg)
            rospy.loginfo(f"Publish pose 2D for: {person_id}")

if __name__ == "__main__":
    try:
        YoloPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass