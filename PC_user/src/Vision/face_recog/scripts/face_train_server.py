#!/usr/bin/env python

import rospy
import os
import numpy as np
import face_recognition
from datetime import datetime
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image

from face_recog.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class TrainingFaceNode:
    def __init__(self):
        rospy.init_node('face_training_node_gpu')
        
        # Configuración de parámetros
        self.train_images_path = rospy.get_param('~train_images_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/')
        self.train_text_path = rospy.get_param('~train_text_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Text/')
        self.min_faces = rospy.get_param('~min_faces', 1)
        self.max_faces = rospy.get_param('~max_faces', 3)
        self.image_quality = rospy.get_param('~image_quality', 95)
        self.model_type = rospy.get_param('~model_type', 'cnn')  # 'cnn' para FaceNet con GPU
        
        # Configuración de GPU
        self.gpu_config()
        
        # Expandir paths de usuario
        self.train_images_path = os.path.expanduser(self.train_images_path)
        self.train_text_path = os.path.expanduser(self.train_text_path)
        
        # Crear directorios si no existen
        os.makedirs(self.train_images_path, exist_ok=True)
        os.makedirs(self.train_text_path, exist_ok=True)
        
        # Inicialización de variables
        self.bridge = CvBridge()
        self.current_image = None
        self.face_encodings = []
        
        # Servicios y suscriptores
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.train_face_service = rospy.Service('/vision/training_face/name', FaceTrainSrv, self.handle_face_training)
        
        rospy.loginfo("Nodo de entrenamiento facial con GPU inicializado")

    def gpu_config(self):
        # Configurar TensorFlow para usar GPU
        gpus = tf.config.experimental.list_physical_devices('GPU')
        print(f"TensorFlow version: {tf.__version__}")
        print(f"GPU disponible: {tf.config.list_physical_devices('GPU')}")
        if gpus:
            try:
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                rospy.loginfo("GPU configurada correctamente")
            except RuntimeError as e:
                rospy.logerr(f"Error al configurar GPU: {e}")

    def image_callback(self, data):   
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error en CvBridge: {e}")
            return

    def handle_face_training(self, req):
        response = FaceTrainSrvResponse()
        
        if self.current_image is None:
            response.success = False
            response.message = "No se ha recibido ninguna imagen de la cámara."
            return response
            
        if not req.name.data.strip():
            response.success = False
            response.message = "El nombre no puede estar vacío."
            return response
            
        try:
            # Detección de caras con FaceNet (modelo CNN)
            face_locations = face_recognition.face_locations(self.current_image, model=self.model_type)
            face_encodings = face_recognition.face_encodings(self.current_image, face_locations)
            
            num_faces = len(face_locations)
            
            if num_faces < self.min_faces:
                response.success = False
                response.message = f"No se detectaron caras. Se requieren al menos {self.min_faces} cara(s)."
                return response
            elif num_faces > self.max_faces:
                response.success = False
                response.message = f"Se detectaron {num_faces} caras. Solo se permite un máximo de {self.max_faces}."
                return response
                
            # Procesamiento de la cara principal
            main_face_encoding = face_encodings[0]
            main_face_location = face_locations[0]
            
            # Generar nombre de archivo único
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename_base = f"{req.name.data}_{timestamp}"
            
            # Guardar imagen con anotaciones
            marked_image = self.current_image.copy()
            top, right, bottom, left = main_face_location
            cv.rectangle(marked_image, (left, top), (right, bottom), (0, 255, 0), 2)
            
            image_path = os.path.join(self.train_images_path, f"{filename_base}.jpg")
            cv.imwrite(image_path, marked_image, [cv.IMWRITE_JPEG_QUALITY, self.image_quality])
            
            # Guardar embedding de FaceNet
            text_path = os.path.join(self.train_text_path, f"{filename_base}.npy")  # Usamos .npy para numpy
            np.save(text_path, main_face_encoding)
            
            response.success = True
            response.message = f"Cara de {req.name.data} guardada exitosamente con FaceNet (GPU)."
            rospy.loginfo(f"Entrenamiento exitoso para: {req.name.data} - Embedding shape: {main_face_encoding.shape}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error en el procesamiento facial: {str(e)}"
            rospy.logerr(f"Error en FaceNet: {e}", exc_info=True)
            
        return response

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        face_recognition_server = TrainingFaceNode()
        face_recognition_server.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Apagando nodo de reconocimiento facial")