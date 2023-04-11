#include "festino_tools/FestinoHRI.h"

bool FestinoHRI::is_node_set = false;
//Speaker
ros::Publisher FestinoHRI::pubSpeaker;

//Leg Finder
ros::Publisher FestinoHRI::pubLegsFinderEnable;
ros::Subscriber FestinoHRI::subLegsFinderFound;

bool FestinoHRI::_legsFound;

//Human Follower
ros::Publisher FestinoHRI::pubHumanFollowerEnable;
ros::Publisher FestinoHRI::pubHumanFollowerStop;

//Pocket Sphinx
ros::Subscriber FestinoHRI::subSprHypothesis;
ros::Publisher FestinoHRI::pubLoadGrammarPocketSphinx;
ros::Publisher FestinoHRI::pubEnableSpeechPocketSphinx;
ros::Publisher FestinoHRI::pubEnableGrammarPocketSphinx;


//Variables for speech
std::string FestinoHRI::_lastRecoSpeech = "";
std::vector<std::string> FestinoHRI::_lastSprHypothesis;
std::vector<float> FestinoHRI::_lastSprConfidences;
bool FestinoHRI::newSprRecognizedReceived = false;

//
//The startSomething functions return inmediately after starting the requested action
//The others, block until the action is finished
//

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoHRI::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoHRI::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoHRI.->Setting ros node..." << std::endl;
    //Speaker
    pubSpeaker = nh->advertise<std_msgs::String>("/speak", 1000);

    //Leg Finder
    pubLegsFinderEnable = nh->advertise<std_msgs::Bool>("/hri/leg_finder/enable", 1);
    subLegsFinderFound = nh->subscribe("/hri/leg_finder/legs_found", 1, &callbackLegsFound);

    //Human Follower
    pubHumanFollowerEnable = nh->advertise<std_msgs::Bool>("/hri/human_following/start_follow", 1);
    pubHumanFollowerStop = nh->advertise<std_msgs::Empty>("/hri/human_following/stop", 1);
    
    //Pocketo Sphinxo
    subSprHypothesis = nh->subscribe("/recognizedSpeech", 1, &FestinoHRI::callbackSprHypothesis);
    //pubLoadGrammarPocketSphinx = nh->advertise<hri_msgs::SphinxSetFile>("/pocketsphinx/set_jsgf", 1);
    //pubEnableSpeechPocketSphinx = nh->advertise<std_msgs::Bool>("/pocketsphinx/mic", 1);
    //pubEnableGrammarPocketSphinx = nh->advertise<hri_msgs::SphinxSetSearch>("/pocketsphinx/set_search", 1);
    return true;
}


//ESTA SE USA
void FestinoHRI::say(std::string strToSay, int timeout)
{
    std::cout << "FestinoHRI.->Saying: " << strToSay << std::endl;
    std_msgs::String voice;
    voice.data = strToSay;
    FestinoHRI::pubSpeaker.publish(voice);
    ros::Duration(timeout, 0).sleep();
}

/*
//Methods for human following
//ESTA SE USA
void FestinoHRI::startFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = true;
    FestinoHRI::pubFollowStartStop.publish(msg);
}

//ESTA SE USA
void FestinoHRI::stopFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = false;
    FestinoHRI::pubFollowStartStop.publish(msg);
}
*/
//ESTA SE USA
void FestinoHRI::enableLegFinder(bool enable)
{
    if(!enable)
    {
        std::cout << "FestinoHRI.->Leg_finder disabled. " << std::endl;
    }
    else
        std::cout << "FestinoHRI.->Leg_finder enabled." << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    FestinoHRI::pubLegsFinderEnable.publish(msg);
}

bool FestinoHRI::frontalLegsFound()
{
    return FestinoHRI::_legsFound;
}

void FestinoHRI::callbackLegsFound(const std_msgs::Bool::ConstPtr& msg)
{
    // std::cout << "FestinoHRI.->Legs found signal received!" << std::endl;
    FestinoHRI::_legsFound = msg->data;
}

//Human Follower
void FestinoHRI::enableHumanFollower(bool enable)
{
    if(!enable)
    {
        std::cout << "FestinoHRI.->Human_follower disabled. " << std::endl;
    }
    else
        std::cout << "FestinoHRI.->Human_follower enabled." << std::endl;

    std_msgs::Bool msg;
    msg.data = enable;
    FestinoHRI::pubHumanFollowerEnable.publish(msg);
}

void FestinoHRI::stopHumanFollower(){
    std_msgs::Empty msg;
    FestinoHRI::pubHumanFollowerStop.publish(msg);
}

//Pocket sphinx

/*
//Load jsgf file
void FestinoHRI::loadGrammarSpeechRecognized(std::string id, std::string grammar){
    hri_msgs::SphinxSetFile msg;
    msg.id = id;
    msg.file_path = grammar;
    pubLoadGrammarPocketSphinx.publish(msg);
}

//Enable mic
void FestinoHRI::enableSpeechRecognized(bool enable){

    std::cout << "FestinoHRI.->Enable grammar: " << enable << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    pubEnableSpeechPocketSphinx.publish(msg);
 
}

//search topic 
void FestinoHRI::enableGrammarSpeechRecognized(std::string id, float recognitionTime){
    hri_msgs::SphinxSetSearch msg;
    msg.search_id = id;
    msg.recognitionTime = recognitionTime;
    pubEnableGrammarPocketSphinx.publish(msg);
}
*/
void FestinoHRI::callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg)
{
    if(msg->hypothesis.size() < 1 || msg->confidences.size() < 1)
    {
        std::cout << "FestinoHRI.->Invalid speech recog hypothesis: msg is empty" << std::endl;
        return;
    }
    _lastRecoSpeech = msg->hypothesis[0];
    _lastSprHypothesis = msg->hypothesis;
    _lastSprConfidences = msg->confidences;
    std::cout << "FestinoHRI.->Last reco speech: " << _lastRecoSpeech << std::endl;
    newSprRecognizedReceived = true;
}


/*
//ESTA SE USA
void FestinoHRI::enableLegFinderRear(bool enable)
{
    if(!enable)
    {

        std::cout << "FestinoHRI.->Leg_finder_rear disabled. " << std::endl;
    }
    else
        std::cout << "FestinoHRI.->Leg_finder_rear enabled." << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    FestinoHRI::pubLegsRearEnable.publish(msg);
}

//ESTA SE USA
void FestinoHRI::getLatestLegsPoses(float &x, float &y)
{
    x = FestinoHRI::lastLegsPoses.point.x;
    y = FestinoHRI::lastLegsPoses.point.y;
}

//ESTA SE USA
void FestinoHRI::getLatestLegsPosesRear(float &x, float &y)
{
    x = FestinoHRI::lastLegsPosesRear.point.x;
    y = FestinoHRI::lastLegsPosesRear.point.y;
}

//ESTA SE USA
void FestinoHRI::callbackLegsFound(const std_msgs::Bool::ConstPtr& msg)
{
    // std::cout << "FestinoHRI.->Legs found signal received!" << std::endl;
    FestinoHRI::_legsFound = msg->data;
}

//ESTA SE USA
void FestinoHRI::callbackLegsRearFound(const std_msgs::Bool::ConstPtr& msg)
{
    //std::cout << "FestinoHRI.->Legs rear found signal received!" << std::endl;
    FestinoHRI::_legsRearFound = msg->data;
}

//ESTA SE USA
void FestinoHRI::callbackLegsPoses(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    FestinoHRI::lastLegsPoses = *msg;
}

//ESTA SE USA
void FestinoHRI::callbackLegsPosesRear(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    FestinoHRI::lastLegsPosesRear = *msg;
}*/

