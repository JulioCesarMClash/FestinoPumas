#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ultralytics import YOLO
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R

class CupTFPublisher:
    def __init__(self):
        rospy.init_node('cup_tf_publisher', anonymous=True)
        
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster()
        
        self.model = YOLO('yolov8n.pt')  # Asegúrate que el modelo reconoce 'cup'
        
        self.yolo_input_size = 640
        self.camera_width = 640
        self.camera_height = 480
        
        self.scale_x = self.camera_width / self.yolo_input_size
        self.scale_y = self.camera_height / self.yolo_input_size
        
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.pc_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pc_callback)
        
        self.current_pc = None
        self.current_image = None
        self.cup_counter = 0

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error de imagen: {e}")

    def pc_callback(self, msg):
        self.current_pc = msg

    def convert_coordinates(self, yolo_x, yolo_y):
        return (
            int(yolo_x * self.scale_x),
            int(yolo_y * self.scale_y)
        )

    def get_3d_point(self, u, v):
        if not self.current_pc or u < 0 or v < 0 or u >= self.camera_width or v >= self.camera_height:
            return None
        try:
            gen = point_cloud2.read_points(self.current_pc,
                                           field_names=("x", "y", "z"),
                                           uvs=[(u, v)],
                                           skip_nans=True)
            point = next(gen, None)
            if point:
                return (point[0], point[1], point[2])
        except Exception as e:
            rospy.logwarn(f"Error en nube de puntos: {e}")
        return None

    def get_pointcloud_neighborhood(self, u, v, window_size=20, max_distance=0.05):
        half_window = window_size // 2
        points = []
        
        central_point = self.get_3d_point(u, v)
        if central_point is None:
            rospy.logwarn("Punto central inválido, no se puede extraer vecindad")
            return np.array(points)
        
        for du in range(-half_window, half_window + 1):
            for dv in range(-half_window, half_window + 1):
                uu = u + du
                vv = v + dv
                
                if 0 <= uu < self.camera_width and 0 <= vv < self.camera_height:
                    try:
                        gen = point_cloud2.read_points(self.current_pc,
                                                       field_names=("x", "y", "z"),
                                                       uvs=[(uu, vv)],
                                                       skip_nans=True)
                        point = next(gen, None)
                        if point:
                            distance = np.linalg.norm(np.array(point) - np.array(central_point))
                            if distance <= max_distance:
                                points.append([point[0], point[1], point[2]])
                    except Exception as e:
                        rospy.logwarn(f"Error leyendo punto ({uu},{vv}): {e}")
        return np.array(points)

    def compute_pca(self, points):
        if len(points) < 3:
            rospy.logwarn("No hay suficientes puntos para PCA")
            return None, None
        pca = PCA(n_components=3)
        pca.fit(points)
        centroid = np.mean(points, axis=0)
        principal_axis = pca.components_[0]
        return centroid, principal_axis

    def vector_to_quaternion(self, vector):
        default_axis = np.array([1,0,0])
        vector_norm = vector / (np.linalg.norm(vector) + 1e-8)
        dot_prod = np.dot(default_axis, vector_norm)
        
        if np.isclose(dot_prod, 1.0):
            return [0, 0, 0, 1]
        if np.isclose(dot_prod, -1.0):
            return [0, 0, 1, 0]  # 180° rotación
        
        axis = np.cross(default_axis, vector_norm)
        axis_norm = axis / (np.linalg.norm(axis) + 1e-8)
        angle = np.arccos(dot_prod)
        
        rot = R.from_rotvec(axis_norm * angle)
        return rot.as_quat()  # [x, y, z, w]

    def process_detections(self):
        if self.current_image is None or self.current_pc is None:
            return
        
        resized = cv2.resize(self.current_image, (self.yolo_input_size, self.yolo_input_size))
        results = self.model(resized, verbose=False, conf=0.5)
        
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = result.names[cls_id]
                
                if class_name != "cup":
                    continue
                
                bbox = box.xyxy[0].cpu().numpy()
                centroid_x = (bbox[0] + bbox[2]) / 2
                centroid_y = (bbox[1] + bbox[3]) / 2
                
                real_u, real_v = self.convert_coordinates(centroid_x, centroid_y)
                points_3d = self.get_pointcloud_neighborhood(real_u, real_v, window_size=20, max_distance=0.05)
                
                if points_3d.shape[0] == 0:
                    rospy.logwarn("No se encontraron puntos válidos en vecindad")
                    continue
                
                centroid_3d, principal_axis = self.compute_pca(points_3d)
                
                if centroid_3d is not None and principal_axis is not None:
                    quat = self.vector_to_quaternion(principal_axis)
                    self.publish_cup_tf(centroid_3d, quat)
                    self.cup_counter += 1

    def publish_cup_tf(self, position, quaternion):
        transform = TransformStamped()
        
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = f"cup_{self.cup_counter % 4}"
        
        transform.transform.translation.x = position[2]  # Ajustar ejes si necesario
        transform.transform.translation.y = -position[0] - 0.045
        transform.transform.translation.z = -position[1]
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(transform)
        rospy.loginfo(f"TF publicado para {transform.child_frame_id} en {position} con orientación {quaternion}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_detections()
            rate.sleep()

if __name__ == '__main__':
    detector = CupTFPublisher()
    detector.run()
