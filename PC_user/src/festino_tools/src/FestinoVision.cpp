#include "festino_tools/FestinoVision.h"

bool FestinoVision::is_node_set = false;

//Open Pose
ros::Subscriber FestinoVision::subPointingHand;
bool FestinoVision::_pointing_hand;

//Face Recog
ros::ServiceClient FestinoVision::cltFindPersons;
ros::ServiceClient FestinoVision::cltTrainPersons;
std::vector<std::string> FestinoVision::_nameRecog(5);

//Detect objects
ros::ServiceClient FestinoVision::cltDetectObjects;
ros::ServiceClient FestinoVision::cltDetectAllObjects;
ros::Publisher FestinoVision::pubObjStartRecog;
ros::Publisher FestinoVision::pubObjStopRecog;
ros::Publisher FestinoVision::pubObjStartWin;
ros::Publisher FestinoVision::pubObjStopWin;

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
    cltFindPersons = nh->serviceClient<act_pln::FaceRecogSrv>("/vision/recognize_face/names");
    cltTrainPersons = nh->serviceClient<act_pln::FaceTrainSrv>("/vision/training_face/name");

    //Detect objects
    cltDetectObjects = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_objs");
    cltDetectAllObjects = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_all_objs");
    pubObjStartWin = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 1);
    pubObjStopWin = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 0);
    pubObjStartRecog = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 1);
    pubObjStopRecog = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 0);

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

//Face Recog
std::vector<std::string> FestinoVision::enableRecogFacesName(bool flag)
{
    act_pln::FaceRecogSrv srv;
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
    act_pln::FaceTrainSrv srv;
    srv.request.name.data = person;

    if (cltTrainPersons.call(srv))
    {
        std::cout << "Success " << srv.response.success << std::endl;
        std::cout << srv.response.message << std::endl;
    }
}

//Detect Obj
void FestinoVision::startObjectFinding()
{
    std_msgs::Bool msg;
    msg.data = true;
    FestinoVision::pubObjStartRecog.publish(msg);
}

void FestinoVision::stopObjectFinding()
{
    std_msgs::Bool msg;
    msg.data = false;
    FestinoVision::pubObjStopRecog.publish(msg);
}

void FestinoVision::startObjectFindingWindow()
{
    std_msgs::Bool msg;
    msg.data = true;
    FestinoVision::pubObjStartWin.publish(msg);
}

void FestinoVision::stopObjectFindingWindow()
{
    std_msgs::Bool msg;
    msg.data = false;
    FestinoVision::pubObjStopWin.publish(msg);
}


bool FestinoVision::detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles)
{
    std::cout << "FestinoVision.->Trying to detect objects... " << std::endl;
    vision_msgs::DetectObjects srv;
    //srv.request.saveFiles = saveFiles;

    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    //Cambiar dirección de topic
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", ros::Duration(1.0));
    srv.request.point_cloud = *ptr;


    if(!cltDetectObjects.call(srv))
    {
        std::cout << std::endl << "Festino::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    recoObjList=srv.response.recog_objects;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Festino::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    
    std::cout << "FestinoVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}

bool FestinoVision::detectAllObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles)
{   

    std::cout << "FestinoVision.->Trying to detect objects... " << std::endl;
    vision_msgs::DetectObjects srv;
    //srv.request.saveFiles = saveFiles;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    //Cambiar dirección de topic
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", ros::Duration(1.0));
    srv.request.point_cloud = *ptr;
    if(!cltDetectAllObjects.call(srv))
    {
        std::cout << std::endl << "Festino::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    recoObjList=srv.response.recog_objects;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Festino::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    std::cout << "FestinoVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}
