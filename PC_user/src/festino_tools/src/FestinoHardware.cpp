#include "festino_tools/FestinoHardware.h"

bool FestinoHardware::is_node_set = false;

ros::Publisher FestinoHardware::pub_digital;
ros::Publisher FestinoHardware::pub_head_orientation;

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoHardware::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoHardware::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoHardware.->Setting ros node..." << std::endl;
    
    FestinoHardware::pub_head_orientation = nh->advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 1000);
    FestinoHardware::pub_digital = nh->advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    
    return true;
}

void FestinoHardware::setHeadOrientation(float yaw, float pitch)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = yaw;
    msg.data[1] = pitch;
    FestinoHardware::pub_head_orientation.publish(msg);
}

void FestinoHardware::setColorLed(std::string colorName)
{
    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};

    if (colorName == "red")
    {
        arr_values.values = {0,0,0,1,0,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "green")
    {
        arr_values.values = {0,0,0,0,1,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "blue")
    {
        arr_values.values = {0,0,0,0,0,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "yellow")
    {
        arr_values.values = {0,0,0,1,1,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "magenta")
    {
        arr_values.values = {0,0,0,1,0,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "turquoise")
    {
        arr_values.values = {0,0,0,0,1,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "white")
    {
        arr_values.values = {0,0,0,1,1,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

}
