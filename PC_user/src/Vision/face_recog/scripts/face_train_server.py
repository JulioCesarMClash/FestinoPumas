#!/usr/bin/env python

import rospy
import os
import numpy as np
import face_recognition

from face_recog.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class TrainingFaceNode:
    def __init__(self):
        rospy.init_node('face_training_node')
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.face_encodings = []
        self.train_face_service = rospy.Service('/vision/training_face/name', FaceTrainSrv, self.handle_face_training)
    

    def image_callback(self, data):   
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

    def handle_face_training(self, req):
        global cv_image
        # Encontrar la cara en la imagen
        face_locations = face_recognition.face_locations(cv_image)
        face_encodings = face_recognition.face_encodings(cv_image, face_locations)

        # Agregar la codificacion de la cara a la lista
        self.face_encodings.extend(face_encodings)

        # Envia la respuesta del servicio
        response = FaceTrainSrvResponse()

        # Guardar la imagen y las codificaciones de la cara en disco
        if len(self.face_encodings) == 1:
            # Guardar la imagen
            cv.imwrite(os.path.expanduser('~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/'+req.name.data+'.jpg'), cv_image)

            # Guardar las codificaciones de la cara en un archivo de texto
            with open(os.path.expanduser('~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Text/'+req.name.data+'.txt'), 'a') as f:
                np.savetxt(f, self.face_encodings[0])

        
            response.success = True
            response.message = "Cara entrenada con exito con el nombre " + req.name.data
        
        else: 

            response.success = False
            response.message = "No hay un invitado o hay mas de uno en la escena. " 

        return response

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    face_recognition_server = TrainingFaceNode()
    face_recognition_server.spin()
