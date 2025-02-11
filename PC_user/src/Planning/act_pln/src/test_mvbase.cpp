#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoVision.h>
#include <cmath>

geometry_msgs::TransformStamped coord_Aruco;
geometry_msgs::TransformStamped coord_cam_robot;
float error_x = 0.0;
float error_y = 0.0;
int aux = 1;

int main(int argc, char **argv)
{
    std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle nh;
    ros::Rate rate(100000);

    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoVision::enableArucoDet(true);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    do
    {
        try
        {
            if (tf_buffer.canTransform("map", "aruco_marker_107", ros::Time(0), ros::Duration(0.1)))
            {
                coord_Aruco = tf_buffer.lookupTransform("map", "aruco_marker_107", ros::Time(0), ros::Duration(0.1));
            }else
            {
                ROS_WARN("Transform for aruco_marker_107 not available");
                continue;
            }
            if (tf_buffer.canTransform("map", "camera_link", ros::Time(0), ros::Duration(0.1)))
            {
                coord_cam_robot = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(0.1));
            }else
            {
                ROS_WARN("Transform for camera_link not available");
                continue;
            }
            std::cout << "Aruco x:" << coord_Aruco.transform.translation.x << std::endl;
            std::cout << "Robot x:" << coord_cam_robot.transform.translation.x << std::endl;
            std::cout << "Aruco - robot:" << abs(coord_Aruco.transform.translation.y - coord_cam_robot.transform.translation.y) << std::endl;
            ros::spinOnce();

            error_x = abs(coord_Aruco.transform.translation.y - coord_cam_robot.transform.translation.y);
            error_y = coord_cam_robot.transform.translation.x- coord_Aruco.transform.translation.x;
            
            FestinoNavigation::move_base(error_x, error_y, 0.0, 0.01);
            ros::spinOnce();

            if ( error_x < 0.145 && abs(error_y) < 0.05)
            {
                FestinoNavigation::move_base(0.0, 0.0, 0.0, 0.01);
                aux = 0;
            }
            
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }   
    }while(aux);
    
    rate.sleep();
    return 0;
}