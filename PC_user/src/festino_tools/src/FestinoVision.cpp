#include "festino_tools/FestinoVision.h"

bool FestinoVision::is_node_set = false;

//Open Pose
ros::Subscriber FestinoVision::subPointingHand;
bool FestinoVision::_pointing_hand;

//Face Recog
ros::ServiceClient FestinoVision::cltFindPersons;
ros::ServiceClient FestinoVision::cltTrainPersons;
ros::ServiceClient FestinoVision::cltArucoTf;

std::vector<std::string> FestinoVision::_nameRecog(5);


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

    //face_recog
    cltFindPersons = nh->serviceClient<vision_msgs::FaceRecogSrv>("/vision/recognize_face/names");
    cltTrainPersons = nh->serviceClient<vision_msgs::FaceTrainSrv>("/vision/training_face/name");
    cltArucoTf = nh->serviceClient<img_proc::Tag_with_tf>("/vision/find_tag");
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


std::vector<std::string> FestinoVision::enableRecogFacesName(bool flag)
{
    vision_msgs::FaceRecogSrv srv;
    srv.request.is_face_recognition_enabled = flag;

    if (cltFindPersons.call(srv))
    {   
        for (int i = 0; i < srv.response.names.size(); i++)
        {
            //std::cout << "entre_3" <<std::endl;
            std::cout << srv.response.names[i] << " ";
        }
        _nameRecog = srv.response.names;
        //std::cout << "lleno" <<std::endl;
        return _nameRecog;
    }

    else
    {
        std::vector<std::string> vector_vacio;
        //std::cout << "vacio" <<std::endl;
        vector_vacio = srv.response.names;
        return vector_vacio;
    }
   
}

void FestinoVision::TrainingPerson(std::string person)
{
    std::cout << "FestinoVision.->Train person: " << person << std::endl;
    vision_msgs::FaceTrainSrv srv;
    srv.request.name.data = person;

    if (cltTrainPersons.call(srv))
    {
        std::cout << "Success " << srv.response.success << std::endl;
        std::cout << srv.response.message << std::endl;
    }
}

void FestinoVision::enableArucoDet(bool flag)
{
    std::cout<< "FestinoVision.-> Detect Aruco Mark with TF" << std::endl;
    img_proc::Tag_with_tf srv;
    srv.request.is_find_tag_enabled = flag;
    if(cltArucoTf.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
}