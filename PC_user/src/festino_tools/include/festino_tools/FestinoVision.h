#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"


class FestinoVision
{
private:
    static bool is_node_set;
    
    //Open Pose  
    static ros::Subscriber subPointingHand;
    static bool _pointing_hand;


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);


    //Open Pose
    static void callbackPointingHand(const std_msgs::Bool::ConstPtr& msg);
    static bool PointingHand();

private:
    //
    
    
};
