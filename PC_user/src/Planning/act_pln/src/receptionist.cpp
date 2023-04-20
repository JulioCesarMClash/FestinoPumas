//libraries for general functions
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"

//Libraries for FestionoTools
#include "festino_tools/FestinoHRI.h"
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
                FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);

                FestinoHRI::say("I will navigate to the entrance door",4);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("entrance_door");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to inspection point" << std::endl;
                
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

                    //***********************************
                    //---Aqui va lo deteccion de caras---
                    //***********************************
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
                if(recogName)
                {
                    FestinoHRI::say("Hello, my name is Festino, please tell me, what is your name",10);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID,GRAMMAR_POCKET_NAMES);
                    FestinoHRI::enableSpeechRecognized(true);
                    nameUnknown = FestinoHRI::lastRecogSpeech();
                }
                else
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    sleep(0.3);
                    FestinoHRI::say("Please tell me, what is your favorite drink", 10);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID,GRAMMAR_POCKET_DRINKS);
                    FestinoHRI::enableSpeechRecognized(true);
                    drinkUnknown = FestinoHRI::lastRecogSpeech();
                }
                
                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;

                state = SM_WAIT_FOR_PRESENTATION;                
                break;
    				
    		case SM_WAIT_FOR_PRESENTATION:
    			std::cout << test << ".-> State SM_WAIT_FOR_NAME: Waiting for the names." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);


                /*if(FestinoHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                {
                    //if(FestinoRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech))
                    //{
                        //if(FestinoRepresentation::receptionistInterpeted(lastInteSpeech, typeOrder, param))
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
                                    if(FestinoHRI::usePocketSphinx)
                                        FestinoHRI::enableGrammarSpeechRecognized(grammarCommandsID, 0);//load the grammar
                                    else
                                        FestinoHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    if(!FestinoHRI::usePocketSphinx)
                                        FestinoHRI::enableSpeechRecognized(true);
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
                                    ss << ", tell me justina yes or justina no";
                                    FestinoHRI::enableSpeechRecognized(false);
                                    FestinoHRI::say(ss.str(), 10);
                                    if(FestinoHRI::usePocketSphinx)
                                        FestinoHRI::enableGrammarSpeechRecognized(grammarCommandsID, 0);//load the grammar
                                    else
                                        FestinoHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    if(!FestinoHRI::usePocketSphinx)
                                        FestinoHRI::enableSpeechRecognized(true);
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                            }
                        //}
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
                            if(FestinoHRI::usePocketSphinx)
                                FestinoHRI::enableGrammarSpeechRecognized(grammarDrinksID, 0);//load the grammar
                            else
                                FestinoHRI::loadGrammarSpeechRecognized(GRAMMAR_DRINKS);
                            if(!FestinoHRI::usePocketSphinx)
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
                }*/
                break;
    				
    		case SM_PRESENTATION_CONFIRM:
    			std::cout << test << ".-> State SM_PRESENTATION_CONFIRM: Confirm presentation." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);

                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                
                if(FestinoHRI::waitForSpecificSentence("robot yes", TIMEOUT_SPEECH))
                {
                    if(lastRecoSpeech.find("yes") != std::string::npos)
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                        {
                            names.push_back(lastName);
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            FestinoHRI::say(ss2.str(), 6);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            drinks.push_back(lastDrink);
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            FestinoHRI::say(ss2.str(), 6);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                    else
                    {
                        if(attemptsConfirmation < MAX_ATTEMPTS_CONFIRMATION)
                        {
                            attemptsConfirmation++;
                            FestinoHRI::enableSpeechRecognized(false);
                            if(recogName)
                            {
                                //names.erase(names.end()- 1);
                                FestinoHRI::say("Sorry I did not understand you, Please tell me what is your name", 7);
                                FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_NAMES);
                                FestinoHRI::enableSpeechRecognized(true);                            
                                nameUnknown = FestinoHRI::lastRecogSpeech();
                            }
                            else
                            {
                                //drinks.erase(names.end() - 1);
                                FestinoHRI::say("Sorry I did not understand you, Please tell me what is your favorite drink", 7);
                                FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                                FestinoHRI::enableSpeechRecognized(true);
                                drinkUnknown = FestinoHRI::lastRecogSpeech();
                            }
                            state = SM_WAIT_FOR_PRESENTATION;
                        }
                        else
                        {
                            FestinoHRI::enableSpeechRecognized(false);
                            if(recogName)
                            {
                                names.push_back(lastName);
                                ss2.str("");
                                ss2 << "Ok, your name is " << names[names.size() - 1];
                                FestinoHRI::say(ss2.str(), 6);
                                FestinoHRI::enableSpeechRecognized(true);
                                recogName = false;
                                state = SM_INTRO_GUEST;
                            }
                            else
                            {
                                drinks.push_back(lastDrink);
                                ss2.str("");
                                ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                                FestinoHRI::say(ss2.str(), 6);
                                attemptsMemorizing = 0;
                                state = SM_MEMORIZING_OPERATOR;
                            }
                        }
                    }
                }
                else
                {
                    if(attemptsWaitConfirmation < MAX_ATTEMPTS_WAIT_CONFIRMATION)
                    {
                        attemptsWaitConfirmation++;
                        FestinoHRI::enableSpeechRecognized(false);
                        FestinoHRI::say(ss.str(), 10);
                        FestinoHRI::enableSpeechRecognized(true);
                        state = SM_PRESENTATION_CONFIRM;
                    }
                    else
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                        {
                            names.push_back(lastName);
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            FestinoHRI::say(ss2.str(), 6);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            drinks.push_back(lastDrink);
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            FestinoHRI::say(ss2.str(), 6);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
    			break;

    		case SM_MEMORIZING_OPERATOR:
    			std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
    			if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING)
                {
                    //JustinaHRI::waitAfterSay("Human, please stay in front of me", 6000, MIN_DELAY_AFTER_SAY);
                    FestinoHRI::say("Human, please not move, and look at me. I'm memorizing your face", 6);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                    //FestinoVision::faceTrain(names[names.size() - 1], 4);

                    // TODO Get service of the face and gender
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                }
                else{
                    memorizingOperators.push_back(false);
                    state = SM_GUIDE_TO_LOC;
                }	
    			break;

 			case SM_WAITING_FOR_MEMORIZING_OPERATOR:
 				std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                /*if(FestinoVision::waitForTrainingFace(TIMEOUT_MEMORIZING)){
                    memorizingOperators.push_back(true);
                    //faces = std::vector<vision_msgs::VisionFaceObject>();
                    faces = JustinaVision::getFaceAgeAndGenderRecognition();
            
                    FestinoTasks::getNearestRecognizedFace(faces.recog_faces, 9.0, centroidFace, gender, "entrance");
                    std::cout << "genderRecog::: " << gender  << " SIZE: "<< faces.recog_faces.size() << std::endl;

                    state = SM_GUIDE_TO_LOC;
                }*/
                attemptsMemorizing++;
    			break;

    		case SM_GUIDE_TO_LOC:
				std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);   			
                FestinoNavigation::moveDistAngle(0, M_PI, 3500);

                //Que navegue a una silla

                //FestinoTasks::guideAPerson(recogLoc, 90000, 1.75);
                attemptsMemorizing = 0;
                findSeatCount = 0;
                FestinoHRI::say("I'm going to find a empty seat for you", 5);
                //JustinaHRI::insertAsyncSpeech("I'm going to find a empty seat for you", 5000, ros::Time::now().sec, 10);
                state = SM_FIND_EMPTY_SEAT;
    			break;

            case SM_NAVIGATE_TO_RECO_LOC:
                std::cout << test << ".-> State SM_NAVIGATE_TO_RECOG_LOC: Navigate to the recog loc." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                //if(!JustinaNavigation::getClose(recogLoc, 80000);
                //    JustinaNavigation::getClose(recogLoc, 80000);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                state = SM_FIND_TO_HOST;
                break;
    				
    		case SM_FIND_TO_HOST:
    			std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                theta = 0;
                //faceCentroids = std::vector<Eigen::Vector3d>();
                //findPerson = JustinaTasks::turnAndRecognizeFace("john", -1, -1, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4 / 2.0, -M_PI_4 / 2.0, 1.0f, 1.0f, faceCentroids, genderRecog, seatPlace);
                if(true)//findPerson)
                {
                    FestinoHRI::say("John, I found you", 3);
                    //JustinaHRI::insertAsyncSpeech("John, I found you", 5000, ros::Time::now().sec, 10);
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    //FestinoTools::transformPoint("/base_link", faceCentroids[0](0, 0), faceCentroids[0](1, 0) , faceCentroids[0](2, 0), "/map", gx_w, gy_w, gz_w);
                    host_z = gz_w;
                    //FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    //FestinoKnowledge::addUpdateKnownLoc("john", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    state = SM_INTRODUCING;
                }
                else
                {
                    if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS)
                    {
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                        findPersonAttemps++;
                    FestinoHRI::say("John, I'm going to find you again", 5);
                    //JustinaHRI::insertAsyncSpeech("John, I'm going to find you again", 5000, ros::Time::now().sec, 10);
                }
    			break;
    				
    		case SM_FIND_TO_GUEST:
				std::cout << test << ".-> State SM_FIND_TO_GUEST: Finding to ." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                theta = 0;
                //faceCentroids = std::vector<Eigen::Vector3d>();
                //findPerson = JustinaTasks::turnAndRecognizeFace(names[names.size() - 1], -1, -1, JustinaTasks::NONE, 0.0f, 0.1f, 0.0f, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, faceCentroids, genderRecog, seatPlace);
                findPerson = true;
                if(findPerson)
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    //FestinoTools::transformPoint("/base_link", faceCentroids[0](0, 0), faceCentroids[0](1, 0) , faceCentroids[0](2, 0), "/map", gx_w, gy_w, gz_w);
                    //guest_z = gz_w;
                    //FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a,120000);
                    //FestinoKnowledge::addUpdateKnownLoc("guest", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    state = SM_NAVIGATE_TO_RECO_LOC;
                }
                else
                {
                    if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS)
                    {
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_NAVIGATE_TO_RECO_LOC;
                    }
                    else
                        findPersonAttemps++;
                    ros::Duration(0.5).sleep(); 
                }
    			break;
    				
    		case SM_INTRODUCING:
    			std::cout << test << ".-> State SM_INTRODUCING: Introducing person to Jhon." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);
                ss.str("");
                if(gender == 1)
                    ss << "John you have a visitor, his name is " << names[names.size() - 1] << " and his favorite drink is " << drinks[drinks.size() - 1];
                else if(gender == 0)
                    ss << "John you have a visitor, her name is " << names[names.size() - 1] << " and her favorite drink is " << drinks[drinks.size() - 1];
                else
                    ss << "John, " << names[names.size() - 1] << " is your visitor, " << names[names.size() - 1] <<  " likes " << drinks[drinks.size() - 1];


                //JustinaHRI::insertAsyncSpeech(ss.str(), 8000, ros::Time::now().sec, 10);
                
                if(true)//FestinoKnowledge::existKnownLocation("john"))
                {
                    //FestinoKnowledge::getKnownLocation("john", goalx, goaly, goala);
                    //Navegar al lugar

                    /*FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a,12000);
                    thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                    
                    if (thetaToGoal < 0.0f)
                        thetaToGoal = 2 * M_PI + thetaToGoal;
                    
                    theta = thetaToGoal - robot_a;
                    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                    
                    FestinoNavigation::moveDistAngle(0, theta, 4000, 12000);
                    FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    
                    dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));
                    
                    */
                    //PREGUNTAR --- NO TENEMOS CABEZA
                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    /*float angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                    if(angleHead < -M_PI)
                        angleHead = 2 * M_PI + angleHead;
                    if(angleHead > M_PI)
                        angleHead = 2 * M_PI - angleHead;*/
                    
                    //FestinoHRI::say(ss.str(), 6);
                    /*if(JustinaKnowledge::existKnownLocation("guest"))
                    {
                        FestinoKnowledge::getKnownLocation("guest", goalx, goaly, goala);
                        FestinoTools::transformPoint("/map", goalx, goaly , guest_z, "/base_link", pointingArmX, pointingArmY, pointingArmZ);
                        if(pointingArmY > 0)
                        {
                            usePointArmLeft = true;
                            FestinoTools::transformPoint("/map", goalx, goaly , guest_z, "/left_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        else
                        {
                            usePointArmLeft = false;
                            FestinoTools::transformPoint("/map", goalx, goaly , guest_z, "/right_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        pointingNormal = sqrt(pointingArmX * pointingArmX + pointingArmY * pointingArmY + pointingArmZ * pointingArmZ);
                        pointingDirX = pointingArmX / pointingNormal;
                        pointingDirY = pointingArmY / pointingNormal;
                        pointingDirZ = pointingArmZ / pointingNormal;
                        pitchAngle = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                        if(pitchAngle <= -M_PI)
                            pitchAngle += 2 * M_PI;
                        else if(pitchAngle >= M_PI)
                            pitchAngle -= 2 * M_PI;
                        
                        //NO HAY BRAZO
                        if(usePointArmLeft)
                        {
                            FestinoManip::laGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            FestinoHRI::say(ss.str(), 6);
                            FestinoManip::startLaGoTo("home");
                        }
                        else
                        {
                            FestinoManip::raGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            FestinoHRI::say(ss.str(), 6);
                            FestinoManip::startRaGoTo("home");
                        }
                    }*/
                }
                ss.str("");
                ss << names[names.size() - 1] << " he is John and his favorite drink is " << hostDrink << std::endl;
                
                //Que navegue al invitado

                //if(FestinoKnowledge::existKnownLocation("guest")){
                    /*f (thetaToGoal < 0.0f)
                        thetaToGoal = 2 * M_PI + thetaToGoal;
                    theta = thetaToGoal - robot_a;
                    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                    
                    FestinoNavigation::moveDistAngle(0, theta, 4000);
                    FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));
                    float torsoSpine, torsoWaist, torsoShoulders;
                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    
                    if(FestinoKnowledge::existKnownLocation("john"))
                    {
                        FestinoKnowledge::getKnownLocation("john", goalx, goaly, goala);
                        FestinoTools::transformPoint("/map", goalx, goaly , host_z, "/base_link", pointingArmX, pointingArmY, pointingArmZ);
                        if(pointingArmY > 0)
                        {
                            usePointArmLeft = true;
                            FestinoTools::transformPoint("/map", goalx, goaly , host_z, "/left_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        else
                        {
                            usePointArmLeft = false;
                            FestinoTools::transformPoint("/map", goalx, goaly , host_z, "/right_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        pointingNormal = sqrt(pointingArmX * pointingArmX + pointingArmY * pointingArmY + pointingArmZ * pointingArmZ);
                        pointingDirX = pointingArmX / pointingNormal;
                        pointingDirY = pointingArmY / pointingNormal;
                        pointingDirZ = pointingArmZ / pointingNormal;
                        pitchAngle = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                        if(pitchAngle <= -M_PI)
                            pitchAngle += 2 * M_PI;
                        else if(pitchAngle >= M_PI)
                            pitchAngle -= 2 * M_PI;
                        if(usePointArmLeft)
                        {
                            JustinaManip::laGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startLaGoTo("home");
                        }
                        else
                        {
                            JustinaManip::raGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startRaGoTo("home");
                        }
                    }*/
                //}
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                if( numGuests++ < 2 )
                    state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                else
                    state = SM_FINISH_TEST;
    			break;
    				
    		case SM_FIND_EMPTY_SEAT:
    			std::cout << test << ".-> State SM_FIND_EMPTY_SEAT: Finding empty seat" << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                if(findSeatCount < MAX_FIND_SEAT_COUNT)
                {
                    //centroids.clear();
                    //findSeat = FestinoTasks::turnAndRecognizeYolo(idsSeat, FestinoTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, seatPlace);
                    if(!findSeat)
                    {
                        findSeatCount++;
                        FestinoHRI::say("I'm going to find a empty seat for you again", 5);
                        //JustinaHRI::insertAsyncSpeech("I'm going to find a empty seat for you again", 5000, ros::Time::now().sec, 10);
                        break;
                    }

                    //centroid = centroids[0];
                    FestinoHRI::say("Please wait", 3);
                    
                    //Aqui se mueve el robot para buscar



                    //FestinoTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                    //FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a,12000);
                    //std::cout << "$$$$$$$$$$$ gx:" << gx_w << " gy :" << gy_w << std::endl;
                    //FestinoKnowledge::addUpdateKnownLoc("guest", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    //goalx = gx_w;
                    //goaly = gy_w;
                    //guest_z = gz_w;
                    //std::cout << "$$$$$$$$$$$ gx:" << gx_w << " gy :" << gy_w << std::endl;
                    //JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 1.2, 30000);
                    //FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a,12000);
                    //thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                    //if (thetaToGoal < 0.0f)
                    //    thetaToGoal += 2 * M_PI;
                    //theta = thetaToGoal - robot_a;
                    //FestinoNavigation::moveDistAngle(0, theta, 3000);
                    //dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly - robot_y, 2));
                    //rate.sleep();
                    //ros::spinOnce();
                    //FestinoNavigation::getRobotPose(robot_x, robot_y, robot_a, 8000);

                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    state = SM_OFFER_EMPTY_SEAT;
                }
                else
                    state = SM_OFFER_EMPTY_SEAT;

    			break;
    				
    		case SM_OFFER_EMPTY_SEAT:
    			std::cout << test << ".-> State SM_OFFER_EMPTY_SEAT: Offer empty seat" << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);
                ss.str("");
                ss << names[names.size() - 1] << ", could you sit in this place, please";

                //JustinaHRI::insertAsyncSpeech(ss.str(), 5000, ros::Time::now().sec, 10);
                /*
                JustinaManip::startLaGoTo("navigation");
                JustinaManip::startRaGoTo("navigation");
                JustinaManip::waitForLaGoalReached(8000);

                JustinaManip::startLaGoTo("offer_seat");
                JustinaManip::startRaGoTo("offer_seat");
                JustinaManip::waitForLaGoalReached(8000);
                */

                FestinoHRI::say(ss.str(), 6);
                //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                

                //ss.str("");
                //ss << names[names.size() - 1] << "Please, look at me";
                //JustinaHRI::waitAfterSay(ss.str(), 4000, MIN_DELAY_AFTER_SAY);
                
                /*
                JustinaManip::startLaGoTo("navigation");
                JustinaManip::startRaGoTo("navigation");
                JustinaManip::waitForLaGoalReached(8000);
                */

                //JustinaHRI::insertAsyncSpeech(ss.str(), 5000, ros::Time::now().sec, 10);


                //JustinaManip::startLaGoTo("home");
                //JustinaManip::startRaGoTo("home");
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                //JustinaHRI::waitAfterSay(ss.str(), 7000, MIN_DELAY_AFTER_SAY);
                state = SM_FIND_TO_GUEST;
    			break;
    				
    		case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I have finished the test", 6);
                success = true;
                
                for(int i = 0; i < names.size(); i++ )
                {
                    
                    std::cout << test << names[i] << std::endl;
                    //FestinoVision::facClearByID(names[i]);
                }
    			break;    				

    	}
    	rate.sleep();
    	ros::spinOnce();
    }
    return 1;
}