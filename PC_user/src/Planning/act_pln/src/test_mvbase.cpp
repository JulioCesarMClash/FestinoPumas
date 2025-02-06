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


int main(int argc, char **argv)
{
    std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);

    FestinoVision::enableArucoDet(true);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    do
    {
        try
        {
            coord_Aruco = tf_buffer.lookupTransform("map", "aruco_marker_107", ros::Time(0), ros::Duration(0.1));
            coord_cam_robot = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(0.2));
            
            std::cout << "Aruco x:" << coord_Aruco.transform.translation.x << std::endl;
            std::cout << "Robot x:" << coord_Aruco.transform.translation.x << std::endl;

            std::cout << "Aruco - robot:" << abs(coord_Aruco.transform.translation.x - coord_cam_robot.transform.translation.x) << std::endl;
            FestinoNavigation::move_base(0.1, 0.0, 0.0, 0.1);
        }
        
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return -1;
        }
    } while (abs(coord_Aruco.transform.translation.x - coord_cam_robot.transform.translation.x) > 0.15);    
    return 0;
}
