#!/usr/bin/env python3

import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
import tf2_ros
import geometry_msgs.msg

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
    
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                rospy.loginfo(f"IDs detected: {ids.flatten()}")

                for i, corner in enumerate(corners):

                    x_min = int(corner[0][:, 0].min())
                    x_max = int(corner[0][:, 0].max())
                    y_min = int(corner[0][:, 1].min())
                    y_max = int(corner[0][:, 1].max())

                    rospy.loginfo(f"x_min: {x_min} -- y_min: {y_min} **** x_max: {x_max} -- y_max:{y_max}")
                    marker_region = cv_image[y_min:y_max, x_min:x_max]

                    cv2.imshow(f"Marker ID {ids[i][0]}", marker_region)
                    cv2.waitKey(1)

            else:
                rospy.loginfo("No ArUco markers in image.")
            
        
        except Exception as e:
            rospy.logerr(f"Error, check camera: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

class ArucoPointCloud:
    def __init__(self):
        rospy.init_node("aruco_pointcloud")

        self.bridge = CvBridge()

        # Diccionario y parámetros de ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()

        # Suscriptores sincronizados
        image_sub = message_filters.Subscriber("/camera/rgb/image_color", Image)
        pointcloud_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, pointcloud_sub], 10, 0.1)
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, rgb_msg, pointcloud_msg):
        try:
            # Convertir imagen RGB a formato OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            # Detectar marcadores ArUco
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                rospy.loginfo(f"IDs detectados: {ids.flatten()}")
                for i, corner in enumerate(corners):
                    # Coordenadas del marcador en píxeles
                    x_min = int(corner[0][:, 0].min())
                    x_max = int(corner[0][:, 0].max())
                    y_min = int(corner[0][:, 1].min())
                    y_max = int(corner[0][:, 1].max())

                    # Extraer puntos de la nube correspondientes a la región del marcador
                    points = []
                    for point in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")):
                        u, v = self.project_to_image(point)
                        if x_min <= u <= x_max and y_min <= v <= y_max:
                            points.append(point)

                    # Convertir a numpy para procesar o guardar
                    points_array = np.array(points)
                    rospy.loginfo(f"Subnube generada para ID {ids[i][0]}: {len(points_array)} puntos.")

                    # Visualizar el rectángulo en la imagen RGB
                    cv2.rectangle(rgb_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Mostrar la imagen RGB con los marcadores detectados
            cv2.imshow("Aruco Detection", rgb_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error procesando las imágenes: {e}")

    def project_to_image(self, point):
        """Proyecta un punto 3D al plano de imagen (u, v) basado en intrínsecos de la cámara."""
        fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5  # Cambiar por los valores intrínsecos de tu cámara
        x, y, z = point
        u = int((x * fx / z) + cx)
        v = int((y * fy / z) + cy)
        return u, v

class ArucoDistance:
    def __init__(self):
        rospy.init_node("aruco_distance")

        self.bridge = CvBridge()

        # Diccionario y parámetros de ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()

        # Suscriptores
        rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_callback)

        # Variables para almacenar datos
        self.latest_image = None
        self.latest_pointcloud = None

    def image_callback(self, msg):
        """Callback para la imagen RGB."""
        self.latest_image = msg

    def pointcloud_callback(self, msg):
        """Callback para la nube de puntos."""
        self.latest_pointcloud = msg

    def process(self):
        """Procesar datos para calcular la distancia."""
        if self.latest_image is None or self.latest_pointcloud is None:
            rospy.loginfo("Esperando datos de imagen y nube de puntos...")
            return

        try:
            # Convertir la imagen RGB a OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            # Detectar marcadores ArUco
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                rospy.loginfo(f"IDs detectados: {ids.flatten()}")
                for i, corner in enumerate(corners):
                    # Calcular el centroide del marcador
                    centroid_x = int(corner[0][:, 0].mean())
                    centroid_y = int(corner[0][:, 1].mean())
                    rospy.loginfo(f"Centroide del marcador {ids[i][0]}: ({centroid_x}, {centroid_y})")

                    # Obtener la coordenada 3D del centroide
                    centroid_3d = self.get_point_from_cloud(centroid_x, centroid_y)

                    if centroid_3d is not None:
                        x, y, z = centroid_3d
                        distance = np.sqrt(x**2 + y**2 + z**2)
                        rospy.loginfo(f"Distancia al marcador {ids[i][0]}: {distance:.3f} m")
                    else:
                        rospy.logwarn(f"No se encontró un punto válido en la nube para el marcador {ids[i][0]}.")

                    # Dibujar el marcador y el centroide en la imagen
                    cv2.circle(rgb_image, (centroid_x, centroid_y), 5, (0, 255, 0), -1)

            # Mostrar la imagen con marcadores detectados
            cv2.imshow("Aruco Detection", rgb_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error procesando los datos: {e}")

    def get_point_from_cloud(self, u, v):
        """Obtener un punto 3D de la nube de puntos basado en coordenadas de imagen."""
        if self.latest_pointcloud is None:
            return None

        # Convertir las coordenadas de la imagen (u, v) en índices de la nube de puntos
        width = self.latest_pointcloud.width
        height = self.latest_pointcloud.height
        index = v * width + u  # Índice lineal en el arreglo de la nube de puntos

        try:
            # Leer el punto en el índice correspondiente
            point_gen = pc2.read_points(self.latest_pointcloud, skip_nans=True, field_names=("x", "y", "z"))
            for i, point in enumerate(point_gen):
                if i == index:
                    x, y, z = point
                    return x, y, z
        except Exception as e:
            rospy.logerr(f"Error accediendo a la nube de puntos: {e}")
            return None

        return None

class ArucoDistanceTF:
    def __init__(self):
        rospy.init_node("aruco_distance_tf")

        self.bridge = CvBridge()

        # Diccionario y parámetros de ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()

        # Suscriptores
        rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_callback)

        # Publicador de TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Variables para almacenar datos
        self.latest_image = None
        self.latest_pointcloud = None

    def image_callback(self, msg):
        """Callback para la imagen RGB."""
        self.latest_image = msg

    def pointcloud_callback(self, msg):
        """Callback para la nube de puntos."""
        self.latest_pointcloud = msg

    def process(self):
        """Procesar datos para calcular la distancia y publicar el TF."""
        if self.latest_image is None or self.latest_pointcloud is None:
            rospy.loginfo("Esperando datos de imagen y nube de puntos...")
            return

        try:
            # Convertir la imagen RGB a OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            # Detectar marcadores ArUco
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                rospy.loginfo(f"IDs detectados: {ids.flatten()}")
                for i, corner in enumerate(corners):
                    # Calcular el centroide del marcador
                    centroid_x = int(corner[0][:, 0].mean())
                    centroid_y = int(corner[0][:, 1].mean())
                    rospy.loginfo(f"Centroide del marcador {ids[i][0]}: ({centroid_x}, {centroid_y})")

                    # Obtener la coordenada 3D del centroide
                    centroid_3d = self.get_point_from_cloud(centroid_x, centroid_y)

                    if centroid_3d is not None:
                        x, y, z = centroid_3d
                        rospy.loginfo(f"Coordenadas 3D del marcador {ids[i][0]}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

                        # Publicar el TF
                        self.publish_tf(x, y, z, ids[i][0])
                    else:
                        rospy.logwarn(f"No se encontró un punto válido en la nube para el marcador {ids[i][0]}.")
                    cv2.circle(rgb_image, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
        
                cv2.imshow("Aruco Detection", rgb_image)
                cv2.waitKey(1)


        except Exception as e:
            rospy.logerr(f"Error procesando los datos: {e}")

    def get_point_from_cloud(self, u, v):
        """Obtener un punto 3D de la nube de puntos basado en coordenadas de imagen."""
        if self.latest_pointcloud is None:
            return None

        # Convertir las coordenadas de la imagen (u, v) en índices de la nube de puntos
        width = self.latest_pointcloud.width
        height = self.latest_pointcloud.height
        index = v * width + u  # Índice lineal en el arreglo de la nube de puntos

        try:
            # Leer el punto en el índice correspondiente
            point_gen = pc2.read_points(self.latest_pointcloud, skip_nans=True, field_names=("x", "y", "z"))
            for i, point in enumerate(point_gen):
                if i == index:
                    x, y, z = point
                    return z, x, abs(y)
        except Exception as e:
            rospy.logerr(f"Error accediendo a la nube de puntos: {e}")
            return None

        return None

    def publish_tf(self, x, y, z, marker_id):       
        """Publicar un TF en la posición del marcador."""
        transform = geometry_msgs.msg.TransformStamped()

        # Configuración del TF
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"  # Cambiar si es necesario
        transform.child_frame_id = f"aruco_marker_{marker_id}"

        # Posición del marcador
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # Orientación (sin rotación, identidad)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Publicar el TF
        self.tf_broadcaster.sendTransform(transform)
        rospy.loginfo(f"Publicado TF para marcador {marker_id} en ({x:.3f}, {y:.3f}, {z:.3f})")


if __name__ == "__main__":
    try:
        node = ArucoDistanceTF()
        rate = rospy.Rate(100000)

        while not rospy.is_shutdown():
            node.process()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
