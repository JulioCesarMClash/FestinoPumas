#include "festino_tools/FestinoVision.h"

bool FestinoVision::is_node_set = false;

//Open Pose
ros::Subscriber FestinoVision::subPointingHand;
bool FestinoVision::_pointing_hand;

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoVision::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoVision.->Setting ros node..." << std::endl;

    //Open Pose
    subPointingHand = nh->subscribe("/vision/pointing_hand/status", 1, &FestinoVision::callbackPointingHand);

    return true;
}


bool FestinoVision::PointingHand()
{
    return _pointing_hand;
}

void FestinoVision::callbackPointingHand(const std_msgs::Bool::ConstPtr& msg)
{
    _pointing_hand = msg -> data; 
}



