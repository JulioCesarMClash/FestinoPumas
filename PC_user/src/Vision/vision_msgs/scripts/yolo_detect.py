#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

current_frame = None
bridge = CvBridge()
model = YOLO('best.pt')

def image_callback(msg):
    global current_frame
    try:
        current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    rospy.init_node('object_detector')
    rospy.Subscriber('/camera/rgb/image_color', Image, image_callback)
    
    cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if current_frame is not None:
            results = model(current_frame, verbose=False)
            annotated_frame = results[0].plot()
            print(results)
            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)
        rate.sleep()
    
    cv2.destroyAllWindows()