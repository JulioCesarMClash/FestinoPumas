#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "robotino_msgs/DigitalReadings.h"

class FestinoHardware
{
private:
    static bool is_node_set;

    //Publisher LED
    static ros::Publisher pub_digital;
    

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for changes led color
    static void setColorLed(std::string colorName);
    
};