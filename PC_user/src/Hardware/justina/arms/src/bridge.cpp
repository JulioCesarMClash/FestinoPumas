#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>  // ¡Nuevo mensaje!
#include <std_msgs/Float64MultiArray.h>

class MoveItToHardwareBridge {
public:
    MoveItToHardwareBridge() {
        ros::NodeHandle nh;
        hardware_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/hardware/left_arm/goal_pose", 10);
        trajectory_sub_ = nh.subscribe("/move_group/display_planned_path", 1, 
                                     &MoveItToHardwareBridge::trajectoryCallback, this);
        ROS_INFO("Puente iniciado. Suscrito a %s", trajectory_sub_.getTopic().c_str());
    }

    void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg) {
        if (msg->trajectory.empty() || msg->trajectory[0].joint_trajectory.points.empty()) {
            ROS_WARN("Trayectoria vacía. Ignorando.");
            return;
        }

        // Extraer la primera trayectoria (puede haber múltiples en DisplayTrajectory)
        const auto& joint_trajectory = msg->trajectory[0].joint_trajectory;
        
        // Depuración: Imprimir joints y posiciones
        ROS_INFO("=== Mensaje recibido ===");
        for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
            ROS_INFO("Joint %s: %f", 
                    joint_trajectory.joint_names[i].c_str(), 
                    joint_trajectory.points[0].positions[i]);
        }

        // Publicar al hardware
        std_msgs::Float64MultiArray array_msg;
        array_msg.data = joint_trajectory.points[0].positions;
        hardware_pub_.publish(array_msg);
        ROS_INFO("Enviado a %s", hardware_pub_.getTopic().c_str());
    }

private:
    ros::Publisher hardware_pub_;
    ros::Subscriber trajectory_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_to_hardware_bridge_cpp");
    MoveItToHardwareBridge bridge;
    ros::spin();
    return 0;
}