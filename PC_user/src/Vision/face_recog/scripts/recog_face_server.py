#!/usr/bin/env python

import rospy
import cv2
import face_recognition
import os

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from face_recog.srv import *

class FaceRecognitionNode:
    def __init__(self):
        rospy.init_node('face_recognition_service')
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_known_faces()
        self.recognize_face_service = rospy.Service('/vision/recognize_face/names', FaceRecogSrv, self.recognize_face)
        self.face_names_recog = []
        
    def load_known_faces(self):
        for filename in os.listdir("/home/juliobotic/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/"):
            name = os.path.splitext(filename)[0]
            image_path = os.path.join("/home/juliobotic/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/", filename)
            image = face_recognition.load_image_file(image_path)
            try:
                face_encoding = face_recognition.face_encodings(image)[0]
            except IndexError:
                print("No hay personas, favor de colocarse enfrente")
            self.known_face_encodings.append(face_encoding)
            self.known_face_names.append(name)

    def image_callback(self, data):
       
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = self.known_face_names[first_match_index]
            face_names.append(name)

        #print(face_names)
        #print(type(face_names))

        self.face_names_recog = face_names

        for (top, right, bottom , left), name in zip(face_locations, face_names):
            cv.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 2)
            cv.putText(cv_image, name, (left, top - 10), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv.imshow("Image Window", cv_image)
        #cv.waitKey(3)

        #print(self.face_names_recog)
        #print(type(self.face_names_recog))

    def recognize_face(self, request):
        
        if request.is_face_recognition_enabled:
            response = FaceRecogSrvResponse()
            response.names = self.face_names_recog
            return response
        else:
            return []

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    face_recognition_node = FaceRecognitionNode()
    face_recognition_node.spin()
