#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pose_estimation/PersonPose3D.h>
#include <pose_estimation/Keypoint3D.h>
#include <unordered_map>

class Pose3DVisualizer {
public:
    Pose3DVisualizer() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        private_nh.param<std::string>("frame_id", frame_id_, "camera_depth_optical_frame");
        
        sub_ = nh.subscribe("/vision/pose_3d", 10, &Pose3DVisualizer::poseCallback, this);
        pub_ = nh.advertise<visualization_msgs::MarkerArray>("/vision/pose_3d/markers", 10);
        
        ROS_INFO("Nodo visualizador 3D iniciado");
    }

    void poseCallback(const pose_estimation::PersonPose3D::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        ros::Time timestamp = ros::Time::now();

        // Mapear keypoints por nombre
        std::unordered_map<std::string, pose_estimation::Keypoint3D> keypoints;
        for (const auto& kp : msg->keypoints) {
            keypoints[kp.name] = kp;
        }

        try {
            // Calcular centroide entre hombros
            const auto& left_shoulder = keypoints.at("shoulder_left");
            const auto& right_shoulder = keypoints.at("shoulder_right");
            
            // Crear marcador para el centroide
            visualization_msgs::Marker centroid_marker;
            centroid_marker.header.frame_id = frame_id_;
            centroid_marker.header.stamp = timestamp;
            centroid_marker.ns = "person_" + std::to_string(msg->id) + "_centroid";
            centroid_marker.id = msg->keypoints.size(); // ID único
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            
            // Posición del centroide
            centroid_marker.pose.position.x = (left_shoulder.x + right_shoulder.x) / 2.0;
            centroid_marker.pose.position.y = (left_shoulder.y + right_shoulder.y) / 2.0;
            centroid_marker.pose.position.z = (left_shoulder.z + right_shoulder.z) / 2.0;
            
            // Propiedades visuales diferentes para el centroide
            centroid_marker.scale.x = 0.1;  // 10cm
            centroid_marker.scale.y = 0.1;
            centroid_marker.scale.z = 0.1;
            centroid_marker.color.r = 0.0;
            centroid_marker.color.g = 1.0;  // Color verde
            centroid_marker.color.b = 0.0;
            centroid_marker.color.a = 1.0;  // Totalmente opaco
            
            centroid_marker.lifetime = ros::Duration(0.2);
            
            // Agregar al array de marcadores
            marker_array.markers.push_back(centroid_marker);
        }
        catch (const std::out_of_range& e) {
            ROS_WARN("No se pueden calcular hombros para el centroide: %s", e.what());
        }

        // Marcadores originales para keypoints
        for (size_t i = 0; i < msg->keypoints.size(); ++i) {
            const auto& kp = msg->keypoints[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = timestamp;
            marker.ns = "person_" + std::to_string(msg->id);
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = kp.x;
            marker.pose.position.y = kp.y;
            marker.pose.position.z = kp.z;
            
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            marker.lifetime = ros::Duration(0.2);
            
            marker_array.markers.push_back(marker);
        }

        pub_.publish(marker_array);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_3d_visualizer");
    Pose3DVisualizer visualizer;
    ros::spin();
    return 0;
}