#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"

#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

//#include "justina_tools/JustinaVision.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#include "robotino_msgs/DigitalReadings.h"

enum SMState
{
    SM_INIT,
    SM_WAIT_FOR_DOOR,
    SM_NAVIGATE_TO_INSPECTION,
    SM_WAITING_FOR_KITCHEN,
    SM_WAIT_FOR_COMMAND,
    SM_REPEAT_COMMAND,
    SM_PARSE_SPOKEN_COMMAND,
    SM_FINAL_STATE,
    SM_WAIT_FOR_CONFIRMATION,
    SM_PARSE_SPOKEN_CONFIRMATION,
    SM_WAIT_FOR_INSPECTION
};

SMState state = SM_INIT;
std::vector<float> goal_vec(3);
std::string location = "nuevito";

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY JULIOSOFT..." << std::endl; //cout
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;

    FestinoHRI::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);
    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000); //, latch=True);
    //FestinoVision::setNodeHandle(&n);
    ros::Rate loop(10);

    //Flags
    bool fail = false;
    bool success = false;
    robotino_msgs::DigitalReadings arr_values;

    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    FestinoHRI::say(" ",3);

    while(ros::ok() && !fail && !success)
    {
        switch(state)
        {
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;
                arr_values.values = {1,1,1,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,1,1,0,0,0};
                pub_digital.publish(arr_values);
                
                FestinoHRI::say("I am ready for robot inspection",3);
                state = SM_WAIT_FOR_DOOR;
                break;

            case SM_WAIT_FOR_DOOR:
                std::cout << "State machine: SM_WAIT_FOR_DOOR" << std::endl;
                arr_values.values = {0,1,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,1,0,0,0,0};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I am waiting for the door to be open",3);
                //if(!FestinoNavigation::obstacleInFront())
                    state = SM_NAVIGATE_TO_INSPECTION;
                break;

            case SM_NAVIGATE_TO_INSPECTION:
                std::cout << "State machine: SM_NAVIGATE_TO_INSPECTION" << std::endl;
                arr_values.values = {0,0,1,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,1,0,0,0};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I can see that the door is open, I am going to inspection point",3);
                sleep(3);
                
                //FestinoNavigation::moveDist(1.0, 4000);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("inspection_point");
                std::cout <<"Coordenates of inspection_point:"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::enl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                    	std::cout << "Cannot move to inspection point" << std::endl;
                FestinoHRI::say("I have arrived to inspection point",1);	
            	sleep(2);
            	FestinoHRI::say("Please, tell me robot yes to go to the exit",3);
            	sleep(2);

     	      	state=SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:                
                std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;
                arr_values.values = {1,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,0,0,0,0,0};
                pub_digital.publish(arr_values);
                FestinoHRI::enableSpeechRecognized(true);
                //ros::Duration(2, 0).sleep();

                if(!FestinoHRI::waitForSpecificSentence("robot yes", 5000))
                {
                    state = SM_WAIT_FOR_COMMAND;
                }
                else
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    std::cout << "Parsing word..." << std::endl;
                    state = SM_PARSE_SPOKEN_COMMAND;
                }
                break;

            case SM_REPEAT_COMMAND:
                arr_values.values = {1,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,0,0,0,0,0};
                pub_digital.publish(arr_values);
                std::cout << "State machine: SM_REPEAT_COMMAND" << std::endl;
                FestinoHRI::say("Please repeat the command",1);
                sleep(2);
                
                state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_PARSE_SPOKEN_COMMAND:
                arr_values.values = {1,0,1,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,0,1,0,0,0};
                pub_digital.publish(arr_values);
                std::cout << "State machine: SM_PARSE_SPOKEN_COMMAND" << std::endl;
                FestinoHRI::say("Please, say continue to confirm the command",3);
                FestinoHRI::enableSpeechRecognized(true);

                if(FestinoHRI::waitForSpecificSentence("continue",5000))
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    sleep(1.5);
                    state = SM_WAIT_FOR_CONFIRMATION;
                }
                else
                {
                    FestinoHRI::say("I can't recognize this command",3);
                    sleep(2);
                    state = SM_REPEAT_COMMAND;
                }
                break;

            case SM_WAIT_FOR_CONFIRMATION:
                std::cout << "State machine: SM_WAIT_FOR_CONFIRMATION" << std::endl;
                arr_values.values = {1,1,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,1,0,0,0,0};
                pub_digital.publish(arr_values);
                //FestinoHRI::say("I am waiting for confirmation",3);
                sleep(2);
                FestinoHRI::enableSpeechRecognized(true);
                if(FestinoHRI::waitForSpecificSentence("robot yes", 9000))
                {  
                        FestinoHRI::enableSpeechRecognized(false);
                        FestinoHRI::say("I am going to continue the robot inspection test",3);
                        sleep(0.5);
                        state = SM_FINAL_STATE;
                        sleep(0.5);
                }
				if(FestinoHRI::waitForSpecificSentence("robot no",6000))
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    state = SM_WAIT_FOR_COMMAND;
                }
                
                break;     
            
            case SM_FINAL_STATE:
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                arr_values.values = {1,1,1,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                arr_values.values = {1,1,1,0,0,0};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I am going to the exit point",3);
                sleep(0.5);

                
                //FestinoNavigation::moveDist(1.0, 4000);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("exit");
                std::cout <<"Coordenates of exit:"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::enl;
               
                if(!JustinaNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 180000))
                    if(!JustinaNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 180000))
                        if(!JustinaNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 180000))
                        success = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}