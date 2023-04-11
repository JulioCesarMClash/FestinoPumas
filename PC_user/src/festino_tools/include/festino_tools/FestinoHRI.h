#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "hri_msgs/RecognizedSpeech.h"
#include "hri_msgs/SphinxSetFile.h"
#include "hri_msgs/SphinxSetSearch.h"
//#include "sound_play/RequestSound.h"
#include "geometry_msgs/PointStamped.h"
//#include "boost/date_time/posix_time/posix_time.hpp"
//#include "boost/thread/thread.hpp"
//#include "bbros_bridge/Default_ROS_BB_Bridge.h"
//#include "hri_msgs/RecognizedSpeech.h"

class FestinoHRI
{
private:
    static bool is_node_set;
    //Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
    //static ros::Publisher pubSay;

    //Speaker  
    static ros::Publisher pubSpeaker;

    //Leg Finder
    static ros::Publisher pubLegsFinderEnable;
    static ros::Subscriber subLegsFinderFound;
    static bool _legsFound;

    //Human Follower
    static ros::Publisher pubHumanFollowerEnable;
    static ros::Publisher pubHumanFollowerStop;
    
    //Pocket Sphinx
    static ros::Subscriber subSprHypothesis;
    static ros::Publisher pubLoadGrammarPocketSphinx;
    static ros::Publisher pubEnableSpeechPocketSphinx;
    static ros::Publisher pubEnableGrammarPocketSphinx; 
    
    static std::string _lastRecoSpeech;
    static std::vector<std::string> _lastSprHypothesis;
    static std::vector<float> _lastSprConfidences;
    static bool newSprRecognizedReceived;


    //static ros::Subscriber subRecognized;
    

    //Variables for speech
    /*static std::string _lastRecoSpeech;
    static std::vector<std::string> _lastSprHypothesis;
    static std::vector<float> _lastSprConfidences;
    static bool newSprRecognizedReceived;*/


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for speech synthesis and recognition
    /*static bool waitForSpeechRecognized(std::string& recognizedSentence, int timeOut_ms);
    static bool waitForSpeechHypothesis(std::vector<std::string>& sentences, std::vector<float>& confidences, int timeOut_ms);
    static bool waitForSpecificSentence(std::string expectedSentence, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string option3,
                                        std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::vector<std::string>& options, std::string& recognized, int timeOut_ms);
    static bool waitForUserConfirmation(bool& confirmation, int timeOut_ms);
    static std::string lastRecogSpeech();*/
    //static void startSay(std::string strToSay);
    //Speaker
    static void say(std::string strToSay, int timeout);

    //Legs Finder
    static void enableLegFinder(bool enable);
    static bool frontalLegsFound();
    static void callbackLegsFound(const std_msgs::Bool::ConstPtr& msg);

    //Human Follower
    static void enableHumanFollower(bool enable);
    static void stopHumanFollower();

    //Pocket Sphinx
    static void loadGrammarSpeechRecognized(std::string id, std::string grammar);
    static void enableSpeechRecognized(bool enable);
    static void enableGrammarSpeechRecognized(std::string id, float recognitionTime);
    
    
    //static void playSound(); 
    /*static void FestinoHRI::startFollowHuman();
    static void FestinoHRI::stopFollowHuman();
    static void FestinoHRI::enableLegFinder(bool enable);
    static void FestinoHRI::enableLegFinderRear(bool enable);
    static bool FestinoHRI::frontalLegsFound();
    static bool FestinoHRI::rearLegsFound();
    static void FestinoHRI::getLatestLegsPoses(float &x, float &y);
    static void FestinoHRI::getLatestLegsPosesRear(float &x, float &y);
    static void FestinoHRI::callbackLegsFound(const std_msgs::Bool::ConstPtr& msg);
    static void FestinoHRI::callbackLegsRearFound(const std_msgs::Bool::ConstPtr& msg);
    static void FestinoHRI::callbackLegsPoses(const geometry_msgs::PointStamped::ConstPtr& msg);
    static void FestinoHRI::callbackLegsPosesRear(const geometry_msgs::PointStamped::ConstPtr& msg);
    static bool FestinoHRI::waitAfterSay(std::string strToSay, int timeout, int delay);*/
private:
    //Speech recog and synthesis
    static void callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg);
    
    
};
