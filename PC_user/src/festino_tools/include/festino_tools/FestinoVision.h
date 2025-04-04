#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "img_proc/FaceRecogSrv.h"
#include "img_proc/FaceTrainSrv.h"
#include "img_proc/Tag_with_tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"


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
    static ros::ServiceClient cltArucoTf;


    //Aruco detector
    static std::vector<std::string> _nameArUcoDet;
    static std::vector<geometry_msgs::PoseStamped> _posArUcoDet;


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);

    //Open Pose
    static void callbackPointingHand(const std_msgs::Bool::ConstPtr& msg);
    static bool PointingHand();

    //Face Recognition
    static std::vector<std::string> enableRecogFacesName(bool flag);
    static void TrainingPerson(std::string person);

    //Aruco detector
    static void enableArucoDet(bool flag);



private:
    //
    
    
};
