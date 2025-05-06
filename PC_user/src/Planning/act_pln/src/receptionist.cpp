//libraries for general functions
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
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
#include "sensor_msgs/LaserScan.h"


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
    SM_FIND_TO_HOST_LOCATE,
    SM_FIND_TO_HOST_CHAIR_B,
    SM_FIND_TO_HOST_CHAIR_A,
    SM_FIND_TO_HOST_SOFA,
    SM_FIND_TO_GUEST,
    SM_INTRODUCING,
    SM_FIND_EMPTY_SEAT,
    SM_OFFER_EMPTY_SEAT,
    SM_FINISH_TEST
};

// Tema
enum Topic
{
    NAME,
    DRINK,
    INTEREST
};

#define MAX_KEYS 10
#define MAX_KEY_LENGTH 50
#define MAX_VALUE_LENGTH 20

bool flag_door = true;
sensor_msgs::LaserScan laserScan;
//Strings aux
std::string nameUnknown;
std::string drinkUnknown;
std::string lastRecoSpeech;
std::string lastInteSpeech;
std::string guestLocation;
std::string auxNames;

std::string test("receptionist");
std::vector<float> goal_vec(3);

// Variables globales
geometry_msgs::Point last_centroid;
bool new_centroid = false;
const double CENTER_THRESHOLD = 10.0; // Umbral en píxeles para considerar centrado
const int IMAGE_CENTER_X = 320;      // Centro horizontal de la imagen (ajustar según resolución)
const int IMAGE_CENTER_Y = 240;      // Centro vertical de la imagen (ajustar según resolución)

// Callback para actualizar el centroide
void centroidCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    last_centroid = *msg;
    new_centroid = true;
    ROS_DEBUG("Centroide actualizado: x=%.2f, y=%.2f", msg->x, msg->y);
}

// Función para verificar si está centrado
bool isCenteredX()
{
    if(!new_centroid) return false;
    
    float error = last_centroid.x - IMAGE_CENTER_X;
    return (fabs(error) < CENTER_THRESHOLD);
}

// Función para mover la cámara (simulada)
void moveCamera(double dx, double dy)
{
    ROS_INFO("Moviendo camara: dx=%.2f, dy=%.2f", dx, dy);
    FestinoNavigation::move_base(0,0,dx*0.02,0.3);
    // Aquí iría el código real para mover la cámara
    // Ejemplo: publicar a un topic de control de PTZ
}

// Función principal de centrado
void centerCamera()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/vision/person_centroid", 1, centroidCallback);
    ros::Rate rate(10); // 10 Hz

    ROS_INFO("Iniciando rutina de centrado de cámara...");

    while (ros::ok())
    {
        ros::spinOnce(); // Procesar callbacks

        if (new_centroid)
        {
            if (isCenteredX())
            {
                ROS_INFO("¡Persona centrada en la imagen!");
                break; // Salir del ciclo cuando esté centrado
            }
            else
            {
                // Calcular movimiento necesario
                double dx = IMAGE_CENTER_X - last_centroid.x;
                double dy = IMAGE_CENTER_Y - last_centroid.y;
                
                // Mover la cámara (proporcional al error)
                moveCamera(dx * 0.1, dy * 0.1); // Factor de ganancia 0.1
            }
            
            new_centroid = false; // Resetear flag
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Esperando datos del centroide...");
        }

        rate.sleep();
    }
}

bool emptyString(const char *str) {
    return (str == NULL || str[0] == '\0');
}

std::string voiceRecognition(const char *words[])
{    
    std::string recog = " ";
    char *token;
    char buffer[256];
    std::string wordFound = "";
    recog = FestinoHRI::lastRecogSpeech();
    sleep(2);
    strcpy(buffer, recog.c_str());
    token = strtok(buffer, " ");
    std::cout << "Escuche->" << token << std::endl;
    for(int i=0; words[i]!=NULL;i++)
    {
        std::cout << "Escuche->" << token <<"-"<<words[i]<<std::endl;
        if (strcmp(token, words[i]) == 0)
        {
            wordFound = token;
        }
    }
    std::cout << "Escuche->" << wordFound <<std::endl;
    return wordFound;
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;

    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    std::cout<<laserScan.ranges.size()<<std::endl;
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 5)
        { 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.5)
    {
        flag_door = true;
        //std::cout<<"door open"<<std::endl;
    }
    else
    {
        flag_door = false;
        //std::cout<<"door closed"<<std::endl;
    }
}


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
    std::vector<std::string> recogPersonAux;
    
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
    int numGuests = 0;

    float pitchAngle;

    int genderRecog;
    int gender = 2;
    char *token;
    char buffer[256];
    bool first = true;
    std::string wordFound = "";
    const char *nombres[] = {"jamie","morgan","michael","jordan","alex","daniel","sergio",NULL};
    const char *bebidas[] = {"coke","soda","tea","water","milk","juice",NULL};
    const char *gustos[] = {"sports","music","movies","reading",NULL};

    std::string param, typeOrder;
    std::string lastName, lastDrink;

    std::vector<std::string> findPersonDetect;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::vector<std::string> topics;
    std::string grammarCommandsID = "receptionisCommands";
    std::string grammarDrinksID = "receptionistDrinks";
    std::string grammarNamesID = "receptionistNames";
    std::string recogLoc = "kitchen";
    std::string seatPlace = "kitchen";
    std::string entranceLoc = "entrance_door";
    std::string hostDrink = "coke";

    names.push_back("john");
    drinks.push_back("coke");    
    topics.push_back("sports");

    // Dictionary dict;
    // init_dict(&dict);
    
    // Añadir elementos
    // dict_add(&dict, "name", "john");
    // dict_add(&dict, "drink", "coke");
    // dict_add(&dict, "topic", "sports");
    
    // Mostrar diccionario
    // dict_print(&dict);

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

    boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;
    
    std::vector<std::string> tokens;

    STATE state = SM_INIT;//SM_SAY_WAIT_FOR_DOOR;
    Topic topic = NAME;

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    robotino_msgs::DigitalReadings arr_values;
    ros::Subscriber subLaserScan = nh.subscribe("/scan", 1, callbackLaserScan);
    ros::Subscriber subCentrodio = nh.subscribe("/vision/pose_3d", 1, callbackLaserScan);
    ros::Publisher pub_digital = nh.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    // ros::Subscriber sub = nh.subscribe("/vision/person_centroid", 10, centroidCallback);
    ros::Rate loop(10);
    
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    FestinoHRI::say(" ",2);
    std::string recog = " ";
    recogName = true;

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
    		case SM_INIT:
    			std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                // arr_values.values = {0,0,0,1,1,1};
                // pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                FestinoHRI::say("I'm ready for receptionist test",3);
                // FestinoHRI::enableSpeechRecognized(false);
                
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                // FestinoHRI::enableSpeechRecognized(false);

                FestinoHRI::say("I will navigate to the entrance door",4);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("entrance_door");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                // if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                //     std::cout << "Cannot move to entrace_door" << std::endl;

                FestinoHRI::say("I have reached the entrance door", 4);


                // std::cout << last_centroid << std::endl;
                centerCamera(); 
                
                if(flag_door)
                //if(true)
                {
                    FestinoHRI::say("Hello human, please enter to the house",6);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    attemptsCheckDoor = 0;
                    // FestinoNavigation::moveDist(-1.3, 300);
                }
                else
                {
                    state = SM_SAY_OPEN_DOOR;
                }
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                // arr_values.values = {0,0,0,0,0,1};
                // pub_digital.publish(arr_values);
                FestinoHRI::say("The door is closed", 3);
                // FestinoNavigation::moveDist(-1.3, 300);
                FestinoHRI::say("Human, can you open the door please", 6);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;   

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << "SM_WAIT_FOR_DOOR" << std::endl;
                // arr_values.values = {0,0,0,1,0,1};
                // pub_digital.publish(arr_values);
                state = SM_SAY_OPEN_DOOR;
                // FestinoHRI::enableSpeechRecognized(false);

                if(flag_door)
                //if(true)
                {
                    FestinoHRI::say("Hello human, can you entrance in the house", 6);
                    //JustinaVision::enableDetectObjsYOLO(true);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
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
                    sleep(5);
                    if(findPersonDetect.size() == 1)
                    {
                        findPersonAttemps = 0;
                        state = SM_INTRO_GUEST;
                        if(findPersonDetect[0] != "unknown" && findPersonDetect[0] !="no_database")
                        {
                            topic = DRINK;
                            strcpy(buffer, findPersonDetect[0].c_str());
                            token = strtok(buffer, " ");
                            names.push_back(token);
                        }
                        else
                        {
                            topic = NAME;
                        }
                    }
                    else
                    {
                        if(findPersonDetect.size() == 0)
                            FestinoHRI::say("Human, i can't find you",3);
                            FestinoHRI::say("Please enter to the house and close the door",6);
                            findPersonAttemps++;

                        if(findPersonDetect.size() > 1)
                            FestinoHRI::say("Human, i see more people",3);
                            FestinoHRI::say("Please, get inside alone and close the door",6);
                            findPersonAttemps++;
                    }
                }
                state = SM_INTRO_GUEST;
                break;
    				
    		case SM_INTRO_GUEST:
    			std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                // arr_values.values = {0,0,0,1,0,1};
                // pub_digital.publish(arr_values);

                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                lastName = "unknown";
                lastDrink = "unknown";

                // FestinoHRI::enableSpeechRecognized(false);

                switch(topic)
                {
                    case NAME:
                        FestinoHRI::say("Nice to meet you, my name is Festino",6);
                        FestinoHRI::say("What is your name?",6);
                        wordFound = voiceRecognition(nombres);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your name?", 8);
                            wordFound = voiceRecognition(nombres);
                            topic = NAME;
                        }
                        names.push_back(wordFound);
                        // dict_add(&dict, "name", wordFound);
                        // topic = DRINK;
                        state = SM_MEMORIZING_OPERATOR;
                        sleep(2);
                        break;

                    case DRINK:
                        ss.str("");
                        ss << "Excuse me "<< names[names.size() - 1] << ", what is your favorite drink?";
                        FestinoHRI::say(ss.str(), 5);
                        wordFound = voiceRecognition(bebidas);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your drink?", 7);
                            wordFound = voiceRecognition(bebidas);
                            topic = DRINK;
                        }
                        drinks.push_back(wordFound);
                        // dict_add(&dict, "drink", wordFound);
                        sleep(2);
                        topic = INTEREST;
                        // state = SM_INTRO_GUEST;
                        break;

                    case INTEREST:
                        ss.str("");
                        ss << names[names.size() - 1] << ", what is your favorite topic?";
                        FestinoHRI::say(ss.str(), 5);
                        wordFound = voiceRecognition(gustos);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your topic?", 7);
                            wordFound = voiceRecognition(gustos);
                            topic = DRINK;
                        }
                        topics.push_back(wordFound);
                        // dict_add(&dict, "topic", wordFound);
                        sleep(2);
                        state = SM_PRESENTATION_CONFIRM;
                        break;
                }
                
                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;

                // state = SM_WAIT_FOR_PRESENTATION;                
                break;
    				
    		case SM_WAIT_FOR_PRESENTATION:
    			std::cout << test << ".-> State SM_WAIT_FOR_PRESENTATION: Waiting for the dates." << std::endl;
                switch(topic)
                {
                    case NAME:
                        FestinoHRI::say("What is your name?",6);
                        wordFound = voiceRecognition(nombres);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your name?", 8);
                            wordFound = voiceRecognition(nombres);
                            topic = NAME;
                        }
                        names.push_back(wordFound);
                        // dict_add(&dict, "name", wordFound);
                        topic = DRINK;
                        sleep(2);
                        break;

                    case DRINK:
                        ss.str("");
                        ss << "Hi "<< names[names.size() - 1] << ", what is your favorite drink?";
                        FestinoHRI::say(ss.str(), 5);
                        wordFound = voiceRecognition(bebidas);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your drink?", 7);
                            wordFound = voiceRecognition(bebidas);
                            topic = DRINK;
                        }
                        drinks.push_back(wordFound);
                        // dict_add(&dict, "drink", wordFound);
                        sleep(2);
                        topic = INTEREST;
                        // state = SM_INTRO_GUEST;
                        break;

                    case INTEREST:
                        ss.str("");
                        ss << names[names.size() - 1] << ", what is your favorite topic?";
                        FestinoHRI::say(ss.str(), 5);
                        wordFound = voiceRecognition(gustos);
                        while(wordFound.empty())
                        {
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your topic?", 7);
                            wordFound = voiceRecognition(gustos);
                            topic = DRINK;
                        }
                        topics.push_back(wordFound);
                        // dict_add(&dict, "topic", wordFound);
                        sleep(2);
                        state = SM_PRESENTATION_CONFIRM;
                        break;
                }
                break;

            case SM_PRESENTATION_CONFIRM:
                std::cout << test << ".-> State SM_PRESENTATION_CONFIRM. Wait for robot yes or robot no" << std::endl;
                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;

                if(first)
                {                    
                    ss2.str("");
                    ss2 << "Your name is " << names[names.size() - 1] << " your favorite drink is " << drinks[drinks.size() - 1] << " and favorite topic is "<< topics[topics.size() - 1];
                    FestinoHRI::say(ss2.str(), 10);
                    FestinoHRI::say("Please confirm your details whit yes or no.",7);
                    first = false;
                }                
                
                recog = FestinoHRI::lastRecogSpeech();
                sleep(2);
                if (recog == "robot yes" || recog == "yes")
                {
                    if(findPersonDetect[0] != "unknown")
                    {
                        state = SM_GUIDE_TO_LOC;
                        attemptsMemorizing++;
                    }
                    else
                    {
                        state = SM_FIND_EMPTY_SEAT;
                    }                    
                }
                else if (recog == "robot no" || recog == "no")
                {
                    state = SM_WAIT_FOR_PRESENTATION;
                    FestinoHRI::say("Understood, let's validate again.",6);               
                }
                else{
                    FestinoHRI::say("Sorry I don't understand you, please speak again.",6);
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;
                ss.str("");
                ss << names[names.size() - 1] << ", please don't move, and look at me. I'm memorizing your face";
                FestinoHRI::say(ss.str(), 6);
                // FestinoHRI::say("Human, please not move, and look at me. I'm memorizing your face", 6);                    
                state = SM_WAITING_FOR_MEMORIZING_OPERATOR;                  
                break;

            case SM_WAITING_FOR_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;                
                if(FestinoVision::TrainingPerson(names[names.size() - 1]))
                {
                    state = SM_GUIDE_TO_LOC;
                    attemptsMemorizing++;
                }
                break;

            case SM_GUIDE_TO_LOC:
                std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;
                FestinoHRI::say("Follow me to the drinks table",3);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("drinks_table");
                std::cout <<"Coordenates of drinks table"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                // if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                //     std::cout << "Cannot move to drinks table" << std::endl;
                
                state = SM_INTRO_GUEST;
                topic = DRINK;
                //Buscar persona
                break;

            case SM_FIND_EMPTY_SEAT:
                std::cout << test << ".-> State SM_FIND_EMPTY_SEAT: Finding empty seat" << std::endl;

                FestinoHRI::say("Follow me to the seats.",3);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_1");
                std::cout <<"Coordenates of chair_1"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                // if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                //     std::cout << "Cannot move to drinks table" << std::endl;

                //Buscar persona en la primera silla

                if(!findSeat)
                {
                    if(findSeatCount < MAX_FIND_SEAT_COUNT)
                    {
                        recogPersonAux.clear();
                        recogPersonAux = FestinoVision::enableRecogFacesName(true);
                        sleep(5);
                        if(recogPersonAux.size() > 0)
                        {
                            ss.str("");
                            ss << "Sorry this place is busy" << recogPersonAux[0] << " is in the area. " << std::endl;
                            findSeat = false;
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            FestinoHRI::say(ss.str(), 5);
                        }
                        else
                        {
                            FestinoHRI::say("Please wait", 3);
                            state = SM_OFFER_EMPTY_SEAT;    
                        }
                    
                    }
                    else
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
                ss << names[names.size() - 1] << ", could you sit in this sofa, please";

                FestinoHRI::say(ss.str(), 6);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                state = SM_FIND_TO_GUEST;
                break;

            case SM_FIND_TO_GUEST:
                std::cout << test << ".-> State SM_FIND_TO_GUEST: Finding to ." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                
                
                findPerson = true;
                if(findPerson)
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
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

            case SM_NAVIGATE_TO_RECO_LOC:
                std::cout << test << ".-> State SM_NAVIGATE_TO_RECOG_LOC: Navigate to the host loc." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("host_loc");
                std::cout <<"Coordenates of John"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to john_location" << std::endl;
                state = SM_FIND_TO_HOST_LOCATE;
                break;

            case SM_FIND_TO_HOST_LOCATE:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in john_location." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                findPersonAttemps++;
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        if(recogPersonAux.size() > 1 && recogPersonAux[1] == "john"){
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            FestinoHRI::say("John, I found you", 3);
                            findPersonCount = 0;
                            findPersonAttemps = 0;
                            findPersonRestart = 0;
                            state = SM_INTRODUCING;
                        }else{
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            FestinoHRI::say("John, I'm going to find you in another site", 5);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to john_location" << std::endl;
                            state = SM_FIND_TO_HOST_CHAIR_B;
                        }
                    }
                }
                else
                {
                    FestinoHRI::say("John, I'm going to find you in another site", 5);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_CHAIR_B;
                }

                break;

            case SM_FIND_TO_HOST_CHAIR_B:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in chairB." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I'm going to find you in another site", 5);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a1");
                        std::cout <<"Coordenates of John"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                        state = SM_FIND_TO_HOST_CHAIR_A;

                    }
                }
                else
                {
                    FestinoHRI::say("John, I'm going to find you in another site", 5);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a1");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_CHAIR_A;
                }
                break;

        case SM_FIND_TO_HOST_CHAIR_A:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in chairA." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
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
                            FestinoHRI::say("John I did not find you, I wll navigate to your chair",3);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("host_loc");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to john_location" << std::endl;
                        }
                        else
                        {
                            FestinoHRI::say("John, I'm going to find you again",3);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("host_loc");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to john_location" << std::endl;
                            state = SM_FIND_TO_HOST_LOCATE;  
                        }

                    }
                }
                else
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    state = SM_INTRODUCING;
                    FestinoHRI::say("John I did not find you, I wll navigate to your chair",3);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("host_loc");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                }
                break;                

            case SM_INTRODUCING:
                std::cout << test << ".-> State SM_INTRODUCING: Introducing person to John." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);

                //**************************************************************
                //---Mover para presentar // Usar lo mismo para ofrecer silla---
                //**************************************************************

                ss.str("");
                if(gender == 1)
                    ss << "John you have a visitor, his name is " << names[names.size() - 1] << " and his favorite drink is " << drinks[drinks.size() - 1];
                else if(gender == 0)
                    ss << "John you have a visitor, her name is " << names[names.size() - 1] << " and her favorite drink is " << drinks[drinks.size() - 1];
                else
                    ss << "John, " << names[names.size() - 1] << " is your visitor, " << names[names.size() - 1] <<  " likes " << drinks[drinks.size() - 1];


                FestinoHRI::say(ss.str(), 5);
                ss.str("");
                goal_vec = FestinoKnowledge::CoordenatesLocSrv(guestLocation);
                std::cout <<"Coordenates of guestLocation"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to gestLocation" << std::endl;
                ss << names[names.size() - 1] << " he is John and his favorite drink is " << hostDrink << std::endl;
                FestinoHRI::say(ss.str(), 5);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                recogName = true;
                if( numGuests++ < 1 )
                    state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                else
                    state = SM_FINISH_TEST;
                break;

            case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("gpsr");
                std::cout <<"Coordenates of finish test"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to finish_test" << std::endl;
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I have finished the test", 6);
                success = true;
                
                for(int i = 0; i < names.size(); i++ )
                {
                    
                    std::cout << test << names[i] << std::endl;
                }
                break;  
                
    	}
    	rate.sleep();
    	ros::spinOnce();
    }
    return 1;
}
