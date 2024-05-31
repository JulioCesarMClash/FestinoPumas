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

#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlanes.h"



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
    
    //Recog objects
    static ros::ServiceClient cltDetectObjects;
    static ros::ServiceClient cltDetectAllObjects;
    static ros::Publisher pubObjStartRecog;
    static ros::Publisher pubObjStopRecog;
    static ros::Publisher pubObjStartWin;
    static ros::Publisher pubObjStopWin;


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);

    //Open Pose
    static void callbackPointingHand(const std_msgs::Bool::ConstPtr& msg);
    static bool PointingHand();

    //Face Recognition
    static std::vector<std::string> enableRecogFacesName(bool flag);
    static void TrainingPerson(std::string person);


    
   
    static void startObjectFinding();
    static void stopObjectFinding();
    static void startObjectFindingWindow();
    static void stopObjectFindingWindow();
    static bool detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles = false);
    static bool detectAllObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles = false);



private:
    //
    
    
};
