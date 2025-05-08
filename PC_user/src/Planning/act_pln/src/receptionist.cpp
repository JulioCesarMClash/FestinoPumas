// Librerías generales
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include <ros/ros.h>

// Librerías de festino
#include <festino_tools/FestinoHRI.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoKnowledge.h>
#include <festino_tools/FestinoHardware.h>

// Parámetros para la prueba
#define EXPECTED_GUESTS 2
#define MAX_FIND_PERSON_RESTART 0
#define MAX_FIND_PERSON_ATTEMPTS 1
#define MAX_ATTEMPTS_MEMORIZING 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 1
#define MAX_FIND_SEAT_COUNT 4

// Estados
enum SMState
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
    SM_BEVERAGE_LOC,
    SM_GUIDE_TO_LOC,
    SM_FIND_TO_HOST_LOCATE,
    SM_FIND_TO_HOST_CHAIR_B,
    SM_FIND_TO_HOST_CHAIR_A,
    SM_FIND_TO_HOST_SOFA,
    SM_FIND_TO_GUEST,
    SM_INTRODUCING,
    SM_FIND_EMPTY_SEAT,
    SM_OFFER_EMPTY_SEAT,
    SM_INTRO_GUEST_TO_GUEST,
    SM_FINISH_TEST
};

// Tema
enum Topic
{
    NAME,
    DRINK,
    INTEREST
};

// VARIABLES GLOBALES
// Banderas
bool success = false;
bool findPerson = false;
bool findSeat = false;
bool recogName = true;

std::vector<bool> memorizingOperators;
std::vector<std::string> recogPersonAux;

// Contadores
int findPersonCount = 0;
int findPersonAttemps = 0;
int findPersonRestart = 0;
int findSeatCount = 0;
int attemptsSpeechReco = 0;
int attemptsSpeechInt = 0;
int attemptsConfirmation = 0;
int attemptsWaitConfirmation = 0;
int attemptsMemorizing = 0;
int numGuests = 0;
int gender = 2;

// Para el movimiento de la cabeza
float pitchAngle;

// Variables auxiliares de navegación
float robot_y, robot_x, robot_a;    
float gx_w, gy_w, gz_w, guest_z, host_z;    
float goalx, goaly, goala;
float dist_to_head;
float theta = 0, thetaToGoal = 0, angleHead = 0;
float pointingArmX, pointingArmY, pointingArmZ;
float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
float distanceArm = 0.6;

std::vector<std::string> findPersonDetect;
std::vector<std::string> names;
std::vector<std::string> drinks;
std::vector<std::string> interests;
std::vector<std::string> tokens;
std::string host_loc = "sofa";
std::string hostName = "jade";
std::string hostDrink = "coke";
std::string hostInterest = "sports";
std::string commands_grammar = "receptionist_commands.json";
std::string names_grammar = "receptionist_names.json";
std::string drinks_grammar = "receptionist_drinks.json";
std::string interests_grammar = "receptionist_interests.json";
std::string left_arm_pose = "default";

//Strings aux
std::string lastRecoSpeech;
std::string guestLocation;
std::string param;
std::string lastName;
std::string lastDrink;

std::stringstream ss;
std::stringstream ss2;

sensor_msgs::LaserScan laserScan;

// Variables de inicio
std::string test("receptionist");
std::vector<float> goal_vec(3);

int main(int argc, char **argv)
{

    Topic topic = NAME;
    SMState state = SM_INIT;

    // ROS init
    std::cout << "INITIALIZING ACT_PLN BY PAREJITASOFT Inc..." << std::endl; //cout
    ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    FestinoHardware::setNodeHandle(&nh);
    ros::Rate loop(10);

    names.push_back(hostName);
    drinks.push_back(hostDrink);
    interests.push_back(hostInterest);

    FestinoHRI::say(" ",2);

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
    		case SM_INIT:
    			std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                ros::Duration(0.5, 0).sleep();

                FestinoHardware::setHeadOrientation(0.0, -0.2);

                FestinoHRI::say("I'm ready for receptionist test",3);
                
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;

                FestinoHRI::say("I will navigate to the entrance door",3);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("entrance_door");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to entrace_door" << std::endl;

                FestinoHRI::say("I have reached the entrance door", 3);

                if(FestinoNavigation::waitForDoor())
                {
                    findPersonAttemps = 0;
                    FestinoHRI::say("Welcome, please come into the house", 4);
                    FestinoNavigation::moveDist(-0.5, 300);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                }
                else
                {
                    state = SM_SAY_OPEN_DOOR;
                }
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                FestinoHRI::say("Welcome, the door is closed", 3);
                FestinoNavigation::moveDist(-1, 300);
                FestinoHRI::say("Could you help me opening the door, please?", 4);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;   

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << "SM_WAIT_FOR_DOOR" << std::endl;
                state = SM_SAY_OPEN_DOOR;

                if(FestinoNavigation::waitForDoor())
                {
                    findPersonAttemps = 0;
                    FestinoHRI::say("Hello, please get inside the house and close the door", 6);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                }
                else
                {
                    state = SM_SAY_OPEN_DOOR;
                }
                break; 

            case SM_WAIT_FOR_PERSON_ENTRANCE:
                std::cout << test << ".-> State SM_WAIT_FOR_PERSON_ENTRANCE: Intro Guest." << std::endl;

                if(findPersonAttemps < MAX_FIND_PERSON_ATTEMPTS)
                {
                    findPersonDetect = FestinoVision::enableRecogFacesName(true);
                    for (int i = 0; i < findPersonDetect.size(); i++)
                        std::cout << findPersonDetect[i] << " " << std::endl;
                    sleep(3);

                    if(findPersonDetect.size() == 1)
                    {
                        findPersonAttemps = 0;
                        
                        if(numGuests != 0)
                        {
                            // Si ya conoce a la persona, la saluda y la pasa a la zona de bebidas
                            if(findPersonDetect[0] == names[0] || findPersonDetect[0] == names[1])
                                state = SM_BEVERAGE_LOC;
                            else
                                state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            state = SM_INTRO_GUEST;
                        }
                    }
                    else
                    {
                        if(findPersonDetect.size() == 0)
                            FestinoHRI::say("Excuse me, i can't find you",3);
                            FestinoHRI::say("Please enter to the house and close the door",6);
                            findPersonAttemps++;

                        if(findPersonDetect.size() > 1)
                            FestinoHRI::say("Excuse me, i see more people",3);
                            FestinoHRI::say("Please, get inside alone and close the door",6);
                            findPersonAttemps++;
                    }
                }
                else
                {
                    findPersonAttemps = 0;
                    topic = NAME;

                    state = SM_INTRO_GUEST;
                }
                break;
    				
    		case SM_INTRO_GUEST:
    			std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;

                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                lastName = "unknown";
                lastDrink = "unknown";

                switch(topic)
                {
                    case NAME:
                        FestinoHRI::say("Nice to meet you, my name is Justina",5);
                        FestinoHRI::say("What is your name?",3);
                        sleep(2);
                        break;

                    case DRINK:
                        ss.str("");
                        ss << names[names.size() - 1] << ", what is your favorite drink?";
                        FestinoHRI::say(ss.str(), 4);
                        sleep(2);
                        break;

                    case INTEREST:
                        ss.str("");
                        ss << names[names.size() - 1] << ", what is your favorite topic?";
                        FestinoHRI::say(ss.str(), 4);
                        sleep(2);
                        break;
                }

                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;

                state = SM_WAIT_FOR_PRESENTATION;                
                break;
    				
    		case SM_WAIT_FOR_PRESENTATION:
    			std::cout << test << ".-> State SM_WAIT_FOR_PRESENTATION: Waiting for the names." << std::endl;

                switch(topic)
                {
                    case NAME:
                        lastRecoSpeech = FestinoHRI::lastRecogSpeech(names_grammar);
                        std::cout << "frase :"<<lastRecoSpeech<<std::endl;
                        names.push_back(lastRecoSpeech);
                        ss.str("");
                        ss << "Did you say?" << names[names.size()-1];
                        FestinoHRI::say(ss.str(), 3);
                        state = SM_PRESENTATION_CONFIRM;
                        break;

                    case DRINK:
                        lastRecoSpeech = FestinoHRI::lastRecogSpeech(drinks_grammar);
                        std::cout << "frase :"<<lastRecoSpeech<<std::endl;
                        drinks.push_back(lastRecoSpeech);
                        ss.str("");
                        ss << "Did you say?" << drinks[drinks.size()-1];
                        FestinoHRI::say(ss.str(), 3);
                        state = SM_PRESENTATION_CONFIRM;
                        break;

                    case INTEREST:
                        lastRecoSpeech = FestinoHRI::lastRecogSpeech(interests_grammar);
                        std::cout << "frase :"<<lastRecoSpeech<<std::endl;
                        interests.push_back(lastRecoSpeech);
                        ss.str("");
                        ss << "Did you say?" << interests[interests.size()-1];
                        FestinoHRI::say(ss.str(), 3);
                        state = SM_PRESENTATION_CONFIRM;
                        break;
                }
                break;

            case SM_PRESENTATION_CONFIRM:
                std::cout << test << ".-> State SM_PRESENTATION_CONFIRM. Wait for robot yes or robot no" << std::endl;

                lastRecoSpeech = "";
                lastRecoSpeech = FestinoHRI::lastRecogSpeech(commands_grammar);

                if (lastRecoSpeech == "justina yes" || lastRecoSpeech == "robot yes" || lastRecoSpeech == "yes")
                {
                    switch(topic)
                    {
                        case NAME:
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            FestinoHRI::say(ss2.str(), 4);
                            topic = DRINK;
                            state = SM_INTRO_GUEST;
                            break;

                        case DRINK:
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            FestinoHRI::say(ss2.str(), 4);
                            topic = INTEREST;
                            state = SM_INTRO_GUEST;
                            break;

                        case INTEREST:
                            ss2.str("");
                            ss2 << "Ok, your favorite topic is " << interests[interests.size() - 1];
                            FestinoHRI::say(ss2.str(), 4);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                            break;
                    }
                }
                else
                {
                    if(attemptsConfirmation < MAX_ATTEMPTS_CONFIRMATION)
                    {
                        switch(topic)
                        {
                            case NAME:
                                FestinoHRI::say("Sorry I don't understand you, Please repeat what is your name", 7);
                                names.pop_back();
                                sleep(2);
                                break;

                            case DRINK:
                                FestinoHRI::say("Sorry I don't understand you, Please repeat what is your favorite drink", 8);
                                drinks.pop_back();
                                sleep(2);
                                break;

                            case INTEREST:
                                FestinoHRI::say("Sorry I don't understand you, Please repeat what is your favorite topic", 8);
                                interests.pop_back();
                                sleep(2);
                                break;
                        }

                        ++attemptsConfirmation;
                        state = SM_WAIT_FOR_PRESENTATION;
                    }
                    else
                    {
                        attemptsConfirmation = 0;

                        switch(topic)
                        {
                            case NAME:
                                names.pop_back();
                                names.push_back("unknown");
                                FestinoHRI::say("Sorry, i couldn't understand you, your name is unknown", 6);
                                topic = DRINK;
                                state = SM_INTRO_GUEST;
                                break;
                        
                            case DRINK:
                                drinks.pop_back();
                                drinks.push_back("unknown");
                                FestinoHRI::say("Sorry, i couldn't understand you, your drink is unknown", 6);                                topic = INTEREST;
                                state = SM_INTRO_GUEST;
                                break;

                            case INTEREST:
                                attemptsMemorizing = 0;
                                interests.pop_back();
                                interests.push_back("unknown");
                                FestinoHRI::say("Sorry, i couldn't understand you, your interest is unknown", 6);
                                state = SM_MEMORIZING_OPERATOR;
                                break;
                        }
                    }
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;

                if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING)
                {
                    std::cout << names[names.size() - 1] << std::endl;

                    if(names[names.size() - 1] == "unknown")
                    {
                        FestinoHRI::say("Human, please don't move, and look at me. I'm memorizing your face", 6);
                    }
                    else
                    {
                        ss.str("");
                        ss << names[names.size() - 1] << ", please don't move, and look at me. I'm memorizing your face";
                        FestinoHRI::say(ss.str(), 6);
                    }
                    sleep(3);
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                }
                else
                {
                    memorizingOperators.push_back(false);
                    state = SM_GUIDE_TO_LOC;
                }   
                break;

            case SM_WAITING_FOR_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;
                
                if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING)
                {
                    std::cout << "Memorizing " << names.back() << "..." << std::endl;   
                    if(FestinoVision::TrainingPerson(names.back()))
                    {
                        // Beverage state
                        state = SM_BEVERAGE_LOC;
                    }
                    else
                    {
                        attemptsMemorizing++;
                    }
                }
                else
                {
                    state = SM_BEVERAGE_LOC;
                }
                break;

            case SM_BEVERAGE_LOC:
                std::cout << test << ".-> State SM_BEVERAGE_LOC: Guests taking their drink." << std::endl;

                attemptsMemorizing = 0;

                FestinoHRI::say("Please, follow me to the beverage area",4);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("beverage_area");
                std::cout <<"Coordenates of beverage_area"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to beverage_area" << std::endl;

                // Señalar hacia la mesa de bebidas Cambiar pose
                left_arm_pose = "pointing_beverage"; 
                FestinoHardware::setArmPose(left_arm_pose);

                ss.str("");
                ss << names[names.size() - 1] << " feel free to take your " << drinks[drinks.size() - 1];
                FestinoHRI::say(ss.str(),4);

                left_arm_pose = "default"; 
                FestinoHardware::setArmPose(left_arm_pose);

                sleep(5);

                state = SM_GUIDE_TO_LOC;
                break;

            case SM_GUIDE_TO_LOC:
                std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;

                findSeatCount = 0;
                findSeat = false;

                FestinoHRI::say("Follow me to the living room",3);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("living_room");
                std::cout <<"Coordenates of living_room"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to living_room" << std::endl;

                FestinoHRI::say("I'm going to find an empty seat for you, please wait", 5);

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                std::cout <<"Coordenates of sofa"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to sofa" << std::endl;

                state = SM_FIND_EMPTY_SEAT;
                break;

            case SM_FIND_EMPTY_SEAT:
                std::cout << test << ".-> State SM_FIND_EMPTY_SEAT: Finding empty seat" << std::endl;

                if(!findSeat)
                {
                    if(findSeatCount < MAX_FIND_SEAT_COUNT)
                    {
                        FestinoHardware::setHeadOrientation(0.0, -0.5);
                        recogPersonAux.clear();
                        recogPersonAux = FestinoVision::enableRecogFacesName(true);
                        sleep(2);

                        if(recogPersonAux.size() >= 2)
                        {
                            findSeat = false;
                            FestinoHRI::say("Sorry, there is no spot in the sofa", 7);
                            ss.str("");

                            ss << "Those are ";
                            for(int i=0; i < recogPersonAux.size() - 1; ++i)
                               ss << recogPersonAux[i] << ", ";
                            ss << " and " << recogPersonAux[recogPersonAux.size() - 1];

                            FestinoHRI::say(ss.str(), 4);
                            FestinoHardware::setHeadOrientation(0.0, -0.2);
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        }
                        else
                        {
                            FestinoHardware::setHeadOrientation(0.0, -0.2);
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            findSeat = true;
                        }
                            
                        if(!findSeat)
                        {
                            ++findSeatCount;
                            FestinoHRI::say("I'm going to find a empty seat for you again on the left chair", 5);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a1");
                            std::cout <<"Coordenates of chair_a"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                std::cout << "Cannot move to chair A" << std::endl;

                            FestinoHardware::setHeadOrientation(0.0, -0.5);
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(true);
                            sleep(2);

                            if(recogPersonAux.size() >= 1)
                            {
                                findSeat = false;
                                ss.str("");
                                ss << "Sorry, in the left chair is " << recogPersonAux[0];
                                FestinoHRI::say(ss.str(), 3);
                                FestinoHardware::setHeadOrientation(0.0, -0.2);
                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            }   
                            else
                            {
                                findSeat = true;
                                FestinoHardware::setHeadOrientation(0.0, -0.2);
                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            }

                            if(!findSeat)
                            {
                                FestinoHRI::say("I'm going to find a empty seat for you again on the right chair", 5);
                                goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b1");
                                std::cout <<"Coordenates of chair_b"<<std::endl;
                                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                                
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                    std::cout << "Cannot move to chair B" << std::endl;

                                FestinoHardware::setHeadOrientation(0.0, -0.5);
                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                                sleep(5);
                                
                                if(recogPersonAux.size() >= 1)
                                {
                                    findSeat = false;
                                    ss.str("");
                                    ss << "Sorry, in the right chair is " << recogPersonAux[0];
                                    FestinoHRI::say(ss.str(), 3);
                                    FestinoHardware::setHeadOrientation(0.0, -0.2);
                                    recogPersonAux.clear();
                                    recogPersonAux = FestinoVision::enableRecogFacesName(false);

                                    FestinoHRI::say("I'm going to find a empty seat for you again", 4);
                                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                                    std::cout <<"Coordenates of sofa"<<std::endl;
                                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                                    
                                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                        std::cout << "Cannot move to sofa" << std::endl;
                                }
                                else
                                {
                                    findSeat = true;
                                    FestinoHardware::setHeadOrientation(0.0, -0.2);
                                    recogPersonAux.clear();
                                    recogPersonAux = FestinoVision::enableRecogFacesName(false);
                                }
                            }
                            else
                            {
                                FestinoHRI::say("Please wait", 3);
                                state = SM_OFFER_EMPTY_SEAT;    
                            }
                        }
                        else
                        {
                            FestinoHRI::say("Please wait", 3);
                            state = SM_OFFER_EMPTY_SEAT;    
                        }
                    
                    }
                    else
                    {
                        FestinoHRI::say("Please wait", 3);
                        state = SM_OFFER_EMPTY_SEAT;
                    }
                }
                else
                {
                    FestinoHRI::say("Please wait", 3);
                    state = SM_OFFER_EMPTY_SEAT;
                }
                
                break;                

            case SM_OFFER_EMPTY_SEAT:
                std::cout << test << ".-> State SM_OFFER_EMPTY_SEAT: Offer empty seat" << std::endl;
                
                left_arm_pose = "pointing_chair"; 
                FestinoHardware::setArmPose(left_arm_pose);

                ss.str("");
                ss << names[names.size() - 1] << ", could you sit here, please?";
                FestinoHRI::say(ss.str(), 5);

                left_arm_pose = "default"; 
                FestinoHardware::setArmPose(left_arm_pose);

                FestinoKnowledge::SetLocation("guest_loc_"+std::to_string(numGuests+1));
                state = SM_NAVIGATE_TO_RECO_LOC;
                break;

            case SM_NAVIGATE_TO_RECO_LOC:
                std::cout << test << ".-> State SM_NAVIGATE_TO_RECOG_LOC: Navigate to the host loc." << std::endl;
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;

                goal_vec = FestinoKnowledge::CoordenatesLocSrv(host_loc);
                std::cout <<"Coordenates of Jade"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to jade_location" << std::endl;
                
                state = SM_FIND_TO_HOST_LOCATE;
                break;

            case SM_FIND_TO_HOST_LOCATE:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to Jade in jade_location." << std::endl;
                findPersonAttemps++;

                FestinoHardware::setHeadOrientation(0.0, -0.5);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);

                sleep(2);

                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == hostName)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        ss.str("");
                        ss << names[0] << ", I found you";
                        FestinoHRI::say(ss.str(), 3);
                        host_loc = "sofa";
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        if(recogPersonAux.size() > 1 && recogPersonAux[1] == hostName){
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            ss.str("");
                            ss << names[0] << ", I found you";
                            FestinoHRI::say(ss.str(), 3);
                            host_loc = "sofa";
                            findPersonCount = 0;
                            findPersonAttemps = 0;
                            findPersonRestart = 0;
                            state = SM_INTRODUCING;
                        }
                        else
                        {
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            ss.str("");
                            ss << names[0] << ", I'm going to find you in another site";
                            FestinoHRI::say(ss.str(), 3);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b1");
                            std::cout <<"Coordenates of Jade"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                std::cout << "Cannot move to jade_location" << std::endl;
                            state = SM_FIND_TO_HOST_CHAIR_B;
                        }
                    }
                }
                else
                {
                    ss.str("");
                    ss << names[0] << ", I'm going to find you in another site";
                    FestinoHRI::say(ss.str(), 3);

                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b1");
                    std::cout <<"Coordenates of Jade"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        std::cout << "Cannot move to jade_location" << std::endl;

                    state = SM_FIND_TO_HOST_CHAIR_B;
                }

                break;

            case SM_FIND_TO_HOST_CHAIR_B:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to Jade in chair B." << std::endl;
                
                FestinoHardware::setHeadOrientation(0.0, -0.5);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                
                sleep(2);

                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == hostName)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        ss.str("");
                        ss << names[0] << ", I found you";
                        FestinoHRI::say(ss.str(), 3);
                        host_loc = "chair_b1";
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        ss.str("");
                        ss << names[0] << ", I'm going to find you in another site";
                        FestinoHRI::say(ss.str(), 3);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a1");
                        std::cout <<"Coordenates of John"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                        state = SM_FIND_TO_HOST_CHAIR_A;

                    }
                }
                else
                {
                    ss.str("");
                    ss << names[0] << ", I'm going to find you in another site";
                    FestinoHRI::say(ss.str(), 3);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a1");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_CHAIR_A;
                }
                break;

        case SM_FIND_TO_HOST_CHAIR_A:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to Jade in chair A." << std::endl;
                
                FestinoHardware::setHeadOrientation(0.0, -0.5);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                
                sleep(2);

                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == hostName)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        ss.str("");
                        ss << names[0] << ", I found you";
                        FestinoHRI::say(ss.str(), 3);
                        host_loc = "chair_a1";
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        std::cout << "SM_FIND_TO_HOST attempts: " << findPersonAttemps << std::endl;

                        if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS)
                        {
                            host_loc = "sofa";
                            findPersonCount = 0;
                            findPersonAttemps = 0;
                            findPersonRestart = 0;
                            state = SM_INTRODUCING;
                            ss.str("");
                            ss << names[0] << ", I did not find you, I will navigate to your chair";
                            FestinoHRI::say(ss.str(), 5);
                            FestinoHardware::setHeadOrientation(0.0, -0.2);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv(host_loc);
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                        }
                        else
                        {
                            ss.str("");
                            ss << names[0] << ", I'm going to find you again";
                            FestinoHRI::say(ss.str(), 5);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                            
                            state = SM_FIND_TO_HOST_LOCATE;  
                        }

                    }
                }
                else
                {
                    host_loc = "sofa";
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    state = SM_INTRODUCING;
                    
                    ss.str("");
                    ss << names[0] << ", I did not find you, I will navigate to your chair";
                    FestinoHRI::say(ss.str(), 5);

                    goal_vec = FestinoKnowledge::CoordenatesLocSrv(host_loc);
                    std::cout <<"Coordenates of Jade"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        std::cout << "Cannot move to jade_location" << std::endl;
                }
                break;                

            case SM_INTRODUCING:
                std::cout << test << ".-> State SM_INTRODUCING: Introducing person to John." << std::endl;

                //**************************************************************
                //---Mover para presentar // Usar lo mismo para ofrecer silla---
                //**************************************************************

                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                topic = NAME;

                FestinoHRI::say(ss.str(), 5);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("guest_loc_"+std::to_string(numGuests+1));
                std::cout <<"Coordenates of guestLocation"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to guestLocation" << std::endl;
                
                ss.str("");
                ss << names[names.size() - 1] << ", the host is " << names[0] << ", his favorite drink is " << drinks[0] << " and he likes " << interests[0] << std::endl;
                FestinoHRI::say(ss.str(), 8);
                sleep(2);ss.str("");

                goal_vec = FestinoKnowledge::CoordenatesLocSrv(host_loc);
                std::cout <<"Coordenates of host location"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to host location" << std::endl;

                ss << names[0] << " the new guest is " << names[names.size() - 1] << ", his favorite drink is " << drinks[drinks.size() - 1] << " and he likes " << interests[interests.size() - 1] << std::endl;
                FestinoHRI::say(ss.str(), 8);

                if(numGuests == 1)
                {
                    std::cout << test << ".-> Guest_to_guest" << std::endl;
                    state = SM_INTRO_GUEST_TO_GUEST;
                    break;
                }

                if(++numGuests < EXPECTED_GUESTS)
                {
                    std::cout << test << ".-> Entrance door" << std::endl;
                    state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                }
                else
                {
                    std::cout << test << ".-> Finish test" << std::endl;
                    state = SM_FINISH_TEST;
                }

                break;

            case SM_INTRO_GUEST_TO_GUEST:
                std::cout << test << ".-> State SM_INTRO_GUEST_TO_GUEST" << std::endl;

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("guest_loc_"+std::to_string(numGuests));
                std::cout <<"Coordenates of guestLocation"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to guestLocation" << std::endl;
                
                ss.str("");
                ss << names[names.size() - 2] << ", the new guest is " << names[names.size()-1] << ", his favorite drink is " << drinks[names.size()-1] << " and he likes " << interests[names.size()-1] << std::endl;
                FestinoHRI::say(ss.str(), 8);
                sleep(2);ss.str("");

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("guest_loc_"+std::to_string(numGuests + 1));
                std::cout <<"Coordenates of guestLocation"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to guestLocation" << std::endl;

                ss << names[names.size()-1] << " let me introduce you " << names[names.size()-2] << ", his favorite drink is " << drinks[drinks.size()-2] << "and he likes " << interests[interests.size()-2] << std::endl;
                FestinoHRI::say(ss.str(), 8);

                state = SM_FINISH_TEST;
                break;

            case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                
                FestinoHardware::setHeadOrientation(0.0, -0.2);

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("beverage_area");
                std::cout <<"Coordenates of finish test"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    std::cout << "Cannot move to finish_test" << std::endl;

                FestinoHRI::say("I have finished the test", 6);
                success = true;
                
                for(int i = 0; i < names.size(); ++i)
                    std::cout << test << names[i] << std::endl;

                break;  
                
    	}

    	rate.sleep();
    	ros::spinOnce();
    }
    return 1;
}