//libraries for general functions
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string.h>
#include <sstream>
#include "ros/ros.h"

//Libraries for FestionoTools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"
#include "robotino_msgs/DigitalReadings.h"
#include "sensor_msgs/LaserScan.h"

#include "std_msgs/Bool.h"
#include "string"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
//#include "boost/date_time/posix_time.hpp"
//#include "boost/thread/thread.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/split.hpp>


//Parameters for the test
#define MAX_FIND_PERSON_COUNT 1
#define MAX_FIND_PERSON_RESTART 0
#define MAX_FIND_PERSON_ATTEMPTS 1
#define MAX_CHECK_DOOR 2
#define TIMEOUT_SPEECH 10000
#define MIN_DELAY_AFTER_SAY 0
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 2
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_ATTEMPTS_MEMORIZING 2
#define MAX_FIND_SEAT_COUNT 4
#define TIMEOUT_MEMORIZING 3000


#define GRAMMAR_POCKET_COMMANDS "grammars/receptionist_commands.jsgf"
#define GRAMMAR_POCKET_DRINKS "grammars/order_drink.jsgf"
#define GRAMMAR_POCKET_NAMES "grammars/people_names.jsgf"
    
//States for the state machine
enum STATE
{
	SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_GOTO_RECEPTIONIST_POINT,
    SM_NAVIGATE_TO_ENTRANCE_DOOR,
    SM_NAVIGATE_TO_RECO_LOC,
    SM_SAY_OPEN_DOOR,
    SM_WAIT_FOR_OPEN_DOOR,
    SM_WAIT_FOR_PERSON_ENTRANCE,
    SM_INTRO_GUEST,
    SM_WAIT_FOR_PRESENTATION,
    SM_PRESENTATION_CONFIRM,
    SM_MEMORIZING_OPERATOR,
    SM_WAITING_FOR_MEMORIZING_OPERATOR,
    SM_GUIDE_TO_LOC,
    SM_FIND_TO_HOST,
    SM_FIND_TO_GUEST,
    SM_INTRODUCING,
    SM_FIND_EMPTY_SEAT,
    SM_OFFER_EMPTY_SEAT,
    SM_FINISH_TEST
};

//Strings aux
std::string nameUnknown;
std::string drinkUnknown;
std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string auxNames;

std::string test("receptionist");
std::vector<float> goal_vec(3);

int main(int argc, char **argv)
{
    std::cout << "INITIALIZING ACT_PLN BY PAREJITASOFT Inc..." << std::endl; //cout
	ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //Aux variables
    //Flags
    bool doorOpenFlag = false;
    bool opened = false;
    bool success = false;
    bool findPerson = false;
    bool findSeat = false;
    bool recogName = false;
    bool completeTrainig = false;
    
    
    std::vector<bool> memorizingOperators;
    
    //Counters
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    int findPersonRestart = 0;
    int findSeatCount = 0;
    int attemptsSpeechReco = 0;
    int attemptsSpeechInt = 0;
    int attemptsConfirmation = 0;
    int attemptsWaitConfirmation = 0;
    int attemptsMemorizing = 0;
    int attemptsCheckDoor = 0;
    int numGuests = 1;

    float pitchAngle;

    int genderRecog;
    int gender = 2;
    

    std::string param, typeOrder;
    std::string lastName, lastDrink;

    std::vector<std::string> findPersonDetect;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string grammarCommandsID = "receptionisCommands";
    std::string grammarDrinksID = "receptionistDrinks";
    std::string grammarNamesID = "receptionistNames";
    std::string recogLoc = "kitchen";
    std::string seatPlace = "kitchen";
    std::string entranceLoc = "entrance_door";
    std::string hostDrink = "coke";

    //Eigen::Vector3d centroid;
    //std::vector<Eigen::Vector3d> faceCentroids;
    //std::vector<Eigen::Vector3d> centroids;    

    std::stringstream ss;
    std::stringstream ss2;

    //Nav aux variables
    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w, guest_z, host_z;    
    float goalx, goaly, goala;
    float dist_to_head;
    float theta = 0, thetaToGoal = 0, angleHead = 0;
    float pointingArmX, pointingArmY, pointingArmZ;
    float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
    float distanceArm = 0.6;
    bool usePointArmLeft = false;

 	/*
 	std::vector<std::string> confirmCommands;
    confirmCommands.push_back("festino yes");
    confirmCommands.push_back("festino no");
    confirmCommands.push_back("robot yes");
    confirmCommands.push_back("robot no");*/

    /*
	std::vector<std::string> idsPerson;
    idsPerson.push_back("person");
    std::vector<std::string> idsSeat;
    idsSeat.push_back("chair");
    idsSeat.push_back("sofa");*/


    boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;
    
    std::vector<std::string> tokens;

    STATE state = SM_INIT;//SM_SAY_WAIT_FOR_DOOR;
    
    //std::vector<vision_msgs::VisionObject> yoloObjects;
 	//std::vector<vision_msgs::VisionFaceObject> facesObject;
    //vision_msgs::VisionFaceObjects faces      

    //Eigen::Vector3d centroidFace = Eigen::Vector3d::Zero();

    //FestinoTools
    //FestinoVision::setNodeHandle(&nh);
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    robotino_msgs::DigitalReadings arr_values;

    ros::Publisher pub_digital = nh.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Rate loop(10);
    
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    FestinoHRI::say(" ",1);

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
    		case SM_INIT:
    			std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                FestinoHRI::say("I'm ready for receptionist test",3);
                //FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                //ros::spinOnce();
                //boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                //FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_COMMANDS);
                //ros::spinOnce();
                //boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                //FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_COMMANDS);
                //ros::spinOnce();
                //boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);

                FestinoHRI::say("I will navigate to the entrance door",4);
                //goal_vec = FestinoKnowledge::CoordenatesLocSrv("entrance_door");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                //if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                //    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                //        std::cout << "Cannot move to inspection point" << std::endl;
                
                FestinoHRI::say("I have reached the entrance door", 4);
                doorOpenFlag = true;
                if(doorOpenFlag)
                {
                    FestinoHRI::say("Hello human, please enter to the house",6);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    attemptsCheckDoor = 0;
                }
                else
                {
                    doorOpenFlag = true;
                    state = SM_SAY_OPEN_DOOR;
                }
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                arr_values.values = {0,0,0,0,0,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("Human, can you open the door please", 6);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;   

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << "SM_WAIT_FOR_DOOR" << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                state = SM_SAY_OPEN_DOOR;

                //*****************************
                //---Aqui va lo de la puerta---
                //*****************************

                if(true)
                {
                    FestinoHRI::say("Hello human, can you entrance in the house please", 6);
                    //JustinaVision::enableDetectObjsYOLO(true);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                }
                break; 

            case SM_WAIT_FOR_PERSON_ENTRANCE:
                std::cout << test << ".-> State SM_WAIT_FOR_PERSON_ENTRANCE: Intro Guest." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);

                if(findPersonAttemps < MAX_FIND_PERSON_COUNT)
                {

                    /*findPersonDetect = FestinoVision::enableRecogFacesName(true);
                    if(findPersonDetect.size() == 0)
                    {
                        findPerson = true;
                    }*/

                    findPerson = true;

                    if(findPerson)
                    {
                        //centroid = centroids[0];
                        findPersonCount++;
                    }
                    if(findPersonCount > MAX_FIND_PERSON_COUNT)
                    {
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;

                        FestinoNavigation::moveDist(-1.0, 3000);
                        state = SM_INTRO_GUEST;
                    }
                    else
                    {
                        if(findPersonRestart > MAX_FIND_PERSON_RESTART)
                        {
                         findPersonCount = 0;
                         findPersonRestart = 0;
                         findPersonAttemps++;
                         FestinoHRI::say("Hello human, please enter to the house",6);
                        }
                        else
                            findPersonRestart++;
                    }
                }
                else
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    recogName = true;
                    state = SM_INTRO_GUEST;
                }
                break;
    				
    		case SM_INTRO_GUEST:
    			std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);

                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                lastName = "unknown";
                lastDrink = "unknown";

                FestinoHRI::enableSpeechRecognized(false);
                recogName = true;

                if(recogName)
                {
                    FestinoHRI::say("Hello, my name is Festino, please tell me, what is your name",5);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID,GRAMMAR_POCKET_NAMES);
                    ros::spinOnce();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(300));
                    FestinoHRI::enableSpeechRecognized(true);
                    ros::spinOnce();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    
                    std::cout << "last sentence: "<<nameUnknown << std::endl;
                    do
                    {
                        lastRecoSpeech = FestinoHRI::lastRecogSpeech();
                    }while(lastRecoSpeech == "");
                    FestinoHRI::enableSpeechRecognized(false);
                }
                else
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    sleep(0.3);
                    FestinoHRI::say("Please tell me, what is your favorite drink", 5);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID,GRAMMAR_POCKET_DRINKS);
                    FestinoHRI::enableSpeechRecognized(true);
                    ros::spinOnce();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(800));
                    lastRecoSpeech = FestinoHRI::lastRecogSpeech();
                    std::cout << "last sentence:" << drinkUnknown << std::endl;
                }
                
                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;

                state = SM_WAIT_FOR_PRESENTATION;                
                break;
    				
    		case SM_WAIT_FOR_PRESENTATION:
    			std::cout << test << ".-> State SM_WAIT_FOR_PRESENTATION: Waiting for the names." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                //if(FestinoHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                //{
                            ss.str("");
                            if(recogName && typeOrder.compare("receptionist_guest_name") == 0)
                            {
                                tokens.clear();
                                if(param.compare(" ") != 0 || param.compare("") != 0)
                                {
                                    ss << "Ok, your name is ";
                                    boost::algorithm::split(tokens, param, boost::algorithm::is_any_of("_"));
                                    ss2.str("");
                                    for(int i = 0; i < tokens.size(); i++)
                                    {
                                        ss << tokens[i] << " ";
                                        ss2 << tokens[i];
                                        if(i < tokens.size() -1)
                                            ss2 << " ";
                                    }
                                    lastName = ss2.str();
                                    //names.push_back(ss2.str());
                                    ss << ", tell me festino yes or festino no";
                                    FestinoHRI::enableSpeechRecognized(false);
                                    FestinoHRI::say(ss.str(), 10);
                                    FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID,GRAMMAR_POCKET_COMMANDS);
                                    ros::spinOnce();
                                    boost::this_thread::sleep(boost::posix_time::milliseconds(800));
                                    FestinoHRI::enableSpeechRecognized(true);

                                    lastRecoSpeech =FestinoHRI::lastRecogSpeech(); 
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                            }
                            if(!recogName && typeOrder.compare("receptionist_favorite_drink") == 0){
                                tokens.clear();
                                if(param.compare(" ") != 0 || param.compare("") != 0){
                                    ss << "Ok, your favorite drink is ";
                                    boost::algorithm::split(tokens, param, boost::algorithm::is_any_of("_"));
                                    ss2.str("");
                                    for(int i = 0; i < tokens.size(); i++){
                                        ss << tokens[i] << " ";
                                        ss2 << tokens[i];
                                        if(i < tokens.size() -1)
                                            ss2 << " ";
                                    }
                                    lastDrink = ss2.str();
                                    //drinks.push_back(ss2.str());
                                    ss << ", tell me robot yes or robot no";
                                    FestinoHRI::enableSpeechRecognized(false);
                                    FestinoHRI::say(ss.str(), 10);
                                    
                                    FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID,GRAMMAR_POCKET_COMMANDS);
                                    FestinoHRI::enableSpeechRecognized(true);
                                    lastRecoSpeech =FestinoHRI::lastRecogSpeech(); 
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                         

                    //}
                    if(attemptsSpeechInt < MAX_ATTEMPTS_SPEECH_INT)
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your name", 7);
                        else
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your favorite drink", 7);
                        attemptsSpeechInt++;
                        FestinoHRI::enableSpeechRecognized(true);
                    }
                    else
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName)
                        {
                            ss2.str("");
                            if(lastName.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, you are an unknown person ";
                            else
                                ss2 << "Ok, your name is " << lastName;
                            FestinoHRI::say(ss2.str(), 12);
                            names.push_back(lastName);
                            recogName = false;
                            FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                            FestinoHRI::enableSpeechRecognized(true);//Enable recognized speech
                            
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            ss2.str("");
                            if(lastDrink.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, your favorite drink is unknown";
                            else
                                ss2 << "Ok, your favorite drink is " << lastDrink;
                            FestinoHRI::say(ss2.str(), 12);
                            drinks.push_back(lastDrink);
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                else
                {
                    if(attemptsSpeechReco < MAX_ATTEMPTS_SPEECH_RECO)
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                            FestinoHRI::say("Please tell me what is your name", 7);
                        else
                            FestinoHRI::say("Please tell me what is your favorite drink", 7);
                        attemptsSpeechReco++;
                        FestinoHRI::enableSpeechRecognized(true);
                    }
                    else
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName)
                        {
                            ss2.str("");
                            if(lastName.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, you are an unknown person ";
                            else
                                ss2 << "Ok, your name is " << lastName;
                            FestinoHRI::say(ss2.str(), 7);
                            names.push_back(lastName);
                            recogName = false;
                            FestinoHRI::enableSpeechRecognized(true);
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            ss2.str("");
                            if(lastDrink.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, your favorite drink is unknown";
                            else
                                ss2 << "Ok, your favorite drink is " << lastDrink;
                            FestinoHRI::say(ss2.str(), 7);
                            drinks.push_back(lastDrink);
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }

                break;
                
    	}
    	rate.sleep();
    	ros::spinOnce();
    }
    return 1;
}