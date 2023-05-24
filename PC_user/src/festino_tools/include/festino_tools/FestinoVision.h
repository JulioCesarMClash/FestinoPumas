#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "act_pln/FaceRecogSrv.h"
#include "act_pln/FaceTrainSrv.h"


class FestinoVision
{
private:
    static bool is_node_set;
    
    //Open Pose  
    static ros::Subscriber subPointingHand;
    static bool _pointing_hand;

    //Face Recognition
    static std::vector<std::string> _nameRecog;
    static ros::ServiceClient cltFindPersons;
    static ros::ServiceClient cltTrainPersons;


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);

    //Open Pose
    static void callbackPointingHand(const std_msgs::Bool::ConstPtr& msg);
    static bool PointingHand();

    //Face Recognition
    static std::vector<std::string> enableRecogFacesName(bool flag);
    static void TrainingPerson(std::string person);



private:
    //
    
    
};
