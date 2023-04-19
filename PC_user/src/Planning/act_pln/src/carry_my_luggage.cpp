//https://github.com/RobotJustina/JUSTINA/blob/develop/catkin_ws/src/planning/act_pln/src/carry_my_luggage.cpp
#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"
#include <algorithm>
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "robotino_msgs/DigitalReadings.h"


enum SMState {
	SM_INIT,
	SM_FIND_BAG,
	SM_WAIT_FOR_BAG,
	SM_FIND_PERSON,
	SM_FOLLOW_OPERATOR,
	SM_WAIT_CONF_CAR,
	SM_LEAVE_BAG,
	SM_FIND_QUEUE,
	SM_FOLLOW_QUEUE,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	int confirm_car_again;
	std::cout << "INITIALIZING CARRY MY LUGGAGE NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;
    FestinoHRI::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);

    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);

    ros::Rate loop(30);

    //Speaker
    std::string voice;

    //Open Pose variable
    //True - Left
    //False - Right
    bool pointing_hand;

    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	    		
	    		arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);

	            ros::Duration(2, 0).sleep();
	            FestinoHRI::enableSpeechRecognized(false);
	            voice = "I am ready for the carry my luggage challenge";
	            FestinoHRI::say(voice, 5);
	    		state = SM_FIND_BAG;
	    		break;
	    	case SM_FIND_BAG:{
	    		std::cout << "State machine: SM_FIND_BAG" << std::endl;
	    		voice = "Please point at the object that you want me to carry";
	    		FestinoHRI::say(voice, 5);
				voice = "Are you pointing at the object? tell me Robot yes or Robot no";
				FestinoHRI::say(voice, 5);

				//Enable speech recognition 
				FestinoHRI::enableSpeechRecognized(true);
	    		ros::Duration(2, 0).sleep();

	    		//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			pointing_hand = FestinoVision::PointingHand();
	    			if (pointing_hand){
	    				FestinoNavigation::moveDistAngle(0.0, -0.7853, 10000);
		    			voice = "Please put the left bag on the platform";
		            	FestinoHRI::say(voice, 10);
	    			}
	    			else{
	    				FestinoNavigation::moveDistAngle(0.0, 0.7853, 10000);
		    			voice = "Please put the right bag on the platform";
		            	FestinoHRI::say(voice, 10);
	    			}
	    			FestinoHRI::enableLegFinder(true);
	    			FestinoHRI::enableSpeechRecognized(false);
	    			state=SM_WAIT_FOR_BAG;
	    		}
	    		break;
	        }
	    	case SM_WAIT_FOR_BAG:
	    		std::cout << "State machine: SM_WAIT_FOR_BAG" << std::endl;	

	    		voice = "Did you put the bag? tell me Robot yes or Robot no";
				FestinoHRI::say(voice, 5);

				//Enable speech recognition
				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			FestinoHRI::enableSpeechRecognized(false);
					state = SM_FIND_PERSON;
	    		}
	    		break;
	    	case SM_FIND_PERSON:
				std::cout << "State machine: SM_FIND_PERSON" << std::endl;	

    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found" << std::endl;

	    			voice = "I can't found you, please stand in front of me";
					FestinoHRI::say(voice, 5);

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{
    				voice = "Say follow me when you are ready";
					FestinoHRI::say(voice, 5);

					//Enable speech recognition
					FestinoHRI::enableSpeechRecognized(true);
					ros::Duration(2, 0).sleep();

					//Waiting for the operator to confirm
	    			if(FestinoHRI::waitForSpecificSentence("follow me", 5000)){
	    				FestinoHRI::enableSpeechRecognized(false);
	    				voice = "I'm going to follow you, please say here is the car if we reached the final destination";
						FestinoHRI::say(voice, 5);
	    				state = SM_FOLLOW_OPERATOR;
	    			}
	    		}
	    		break;
	    	case SM_FOLLOW_OPERATOR:
	    		std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;	
	    		
	    		if(confirm_car_again){
	    			voice = "Please say here is the car if we reached the final destination";
	    			FestinoHRI::say(voice, 5);
	    			confirm_car_again = false;
	    		}
	    		
				FestinoHRI::enableHumanFollower(true);

	    		FestinoHRI::enableSpeechRecognized(true);

	    		if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Lost operator" << std::endl;
	    			voice = "I lost you";
					FestinoHRI::say(voice, 5);
	    			FestinoHRI::enableHumanFollower(false);
	    			state = SM_FIND_PERSON;
	    		}
    			
    			if(FestinoHRI::waitForSpecificSentence("here is the car", 5000)){
    				FestinoHRI::enableSpeechRecognized(false);
    				state = SM_WAIT_CONF_CAR;
    			}
    			
	    		break;
	    	case SM_WAIT_CONF_CAR:
				voice = "Is this the car?, say Robot yes or Robot no";
				FestinoHRI::say(voice, 5);

				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
					std::cout << "Here is the car" << std::endl;
					FestinoHRI::enableSpeechRecognized(false);
					FestinoHRI::enableLegFinder(false);
					FestinoHRI::enableHumanFollower(false);
					state = SM_LEAVE_BAG;
				}
				else{
					confirm_car_again = true;
					state = SM_FOLLOW_OPERATOR;
				}

				break;
	    	case SM_LEAVE_BAG:
	    		std::cout << "State machine: SM_LEAVE_BAG" << std::endl;	
	    		voice = "Please take the bag";
	    		FestinoHRI::say(voice, 10);
	    		voice = "Did you take the bag?, say Robot yes or Robot no";
				FestinoHRI::say(voice, 5);

				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			FestinoHRI::enableSpeechRecognized(false);
					state = SM_FIND_QUEUE;
	    		}
	    		break;
	    	case SM_FIND_QUEUE:
	    		std::cout << "State machine: SM_FIND_QUEUE" << std::endl;

	    		//ros::Duration(2, 0).sleep();
	    		FestinoNavigation::moveDistAngle(0.0, 3.1415, 10000);


    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found" << std::endl;

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{
	    			state = SM_FOLLOW_QUEUE;
	    		}
	    		break;
	    	case SM_FOLLOW_QUEUE:
	    		std::cout << "State machine: SM_FOLLOW_QUEUE" << std::endl;

	    		FestinoHRI::enableHumanFollower(true);
	    		ros::Duration(20, 0).sleep();
	    		FestinoHRI::enableHumanFollower(false);

	    		state = SM_FINAL_STATE;

	    		break;
	    	case SM_FINAL_STATE:
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished the test";
	            ros::Duration(5, 0).sleep();
	            FestinoHRI::say(voice, 2);
	    		success = true;
	    		fail = true;
	    		break;
		}
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}