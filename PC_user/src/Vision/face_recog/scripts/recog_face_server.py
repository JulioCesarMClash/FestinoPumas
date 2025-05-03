#!/usr/bin/env python3

import rospy
import cv2
import face_recognition
import os
import numpy as np
from datetime import datetime
import tensorflow as tf

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from face_recog.srv import *

class FaceRecognitionNode:
    def __init__(self):
        rospy.init_node('face_recognition_service_gpu')
        
        # Configuración de parámetros
        self.train_images_path = rospy.get_param('~train_images_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Image/')
        self.train_text_path = rospy.get_param('~train_text_path', '~/FestinoPumas/PC_user/src/Vision/face_recog/Train_faces/Text/')
        self.tolerance = rospy.get_param('~tolerance', 0.6)  # Umbral de reconocimiento
        self.model_type = rospy.get_param('~model_type', 'cnn')  # Usar CNN para FaceNet con GPU
        
        # Configuración de GPU
        self.gpu_config()
        
        # Expandir paths de usuario
        self.train_images_path = os.path.expanduser(self.train_images_path)
        self.train_text_path = os.path.expanduser(self.train_text_path)
        
        # Inicialización de variables
        self.bridge = CvBridge()
        self.current_image = None
        self.known_face_encodings = []
        self.known_face_names = []
        
        # Cargar caras conocidas
        self.load_known_faces()
        
        # Servicios y suscriptores
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.recognize_face_service = rospy.Service('/vision/recognize_face/names', FaceRecogSrv, self.recognize_face)
        self.result_pub = rospy.Publisher('/face_recognition/result', String, queue_size=10)
        
        rospy.loginfo("Nodo de reconocimiento facial con GPU inicializado")

    def gpu_config(self):
        # Configurar TensorFlow para usar GPU
        gpus = tf.config.experimental.list_physical_devices('GPU')
        if gpus:
            try:
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                rospy.loginfo("GPU configurada correctamente")
            except RuntimeError as e:
                rospy.logerr(f"Error al configurar GPU: {e}")

    def load_known_faces(self):
        """Carga los embeddings faciales previamente entrenados"""
        start_time = datetime.now()
        
        # Cargar desde archivos .npy
        for filename in os.listdir(self.train_text_path):
            if filename.endswith('.npy'):
                name = os.path.splitext(filename)[0].split('_')[0]  # Extraer nombre base
                encoding_path = os.path.join(self.train_text_path, filename)
                
                try:
                    face_encoding = np.load(encoding_path)
                    self.known_face_encodings.append(face_encoding)
                    self.known_face_names.append(name)
                    rospy.loginfo(f"Cargada cara conocida: {name}")
                except Exception as e:
                    rospy.logerr(f"Error al cargar encoding para {name}: {e}")
        
        rospy.loginfo(f"Cargadas {len(self.known_face_names)} caras conocidas en {(datetime.now() - start_time).total_seconds():.2f}s")

    def image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error en CvBridge: {e}")
            return

    def recognize_face(self, request):
        response = FaceRecogSrvResponse()
        
        if not request.is_face_recognition_enabled:
            response.names = ["disabled"]
            return response
            
        if self.current_image is None:
            response.names = ["no_image"]
            rospy.logwarn("No hay imagen disponible para reconocimiento")
            return response
            
        try:
            start_time = datetime.now()
            
            # Convertir imagen a RGB (FaceNet espera este formato)
            rgb_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB)
            
            # Detectar caras con FaceNet (GPU)
            face_locations = face_recognition.face_locations(rgb_image, model=self.model_type)
            face_encodings = face_recognition.face_encodings(rgb_image, face_locations)
            
            rospy.logdebug(f"Detección completada en {(datetime.now() - start_time).total_seconds():.2f}s")
            
            # Reconocimiento facial
            face_names = []
            for face_encoding in face_encodings:
                # Comparar con caras conocidas usando distancia euclidiana
                matches = face_recognition.compare_faces(
                    self.known_face_encodings, 
                    face_encoding,
                    tolerance=self.tolerance
                )
                
                # Calcular distancias para mejor precisión
                face_distances = face_recognition.face_distance(
                    self.known_face_encodings, 
                    face_encoding
                )
                
                # Seleccionar la mejor coincidencia
                best_match_index = np.argmin(face_distances)
                name = "Unknown"
                confidence = 1.0 - face_distances[best_match_index]
                
                if matches[best_match_index] and confidence > 0.5:  # Umbral de confianza
                    name = f"{self.known_face_names[best_match_index]} ({confidence:.2f})"
                
                face_names.append(name)
                rospy.loginfo(f"Reconocido: {name} con confianza {confidence:.2f}")
            
            # Publicar resultados
            self.result_pub.publish(String(",".join(face_names)))
            
            # Preparar respuesta
            response.names = face_names
            rospy.loginfo(f"Reconocimiento completado en {(datetime.now() - start_time).total_seconds():.2f}s")
            
            # Visualización (opcional)
            # self.draw_recognitions(rgb_image, face_locations, face_names)
            
        except Exception as e:
            rospy.logerr(f"Error en reconocimiento facial: {e}", exc_info=True)
            response.names = ["error"]
            
        return response

    def draw_recognitions(self, image, face_locations, face_names):
        """Dibuja los resultados del reconocimiento en la imagen"""
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Dibujar rectángulo alrededor del rostro
            cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
            
            # Dibujar etiqueta con nombre
            cv2.rectangle(image, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(image, name, (left + 6, bottom - 6), font, 0.8, (0, 0, 0), 1)
        
        # Mostrar imagen (opcional)
        cv2.imshow('Face Recognition', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        face_recognition_node = FaceRecognitionNode()
        rospy.loginfo("Servicio de reconocimiento facial listo")
        face_recognition_node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Apagando nodo de reconocimiento facial")
        cv2.destroyAllWindows()