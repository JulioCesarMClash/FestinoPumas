//Drive to 12 randomly generated target zones.
//Refbox sends all 12 zones at once.
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

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_WAIT_FOR_BAG,
	SM_FIND_BAG,
	SM_FIND_PERSON,
	SM_WAIT_FOR_ZONES,
	SM_CALC_EU_DIST,
	SM_NAV_NEAREST_ZONE,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;
/*bool flag_zones = false;
std::vector<std_msgs::String> target_zones;
std::vector<geometry_msgs::PoseStamped> tf_target_zones;
std_msgs::String new_zone;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;*/


/*
void callback_refbox_zones(const std_msgs::String::ConstPtr& msg)
{
    new_zone = *msg;
    target_zones.push_back(new_zone);

    if(target_zones.size() == 12){
        flag_zones = true;
    }
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}*/

/*void wait_conformation(msg)
{
    std::cout << msg << std::endl;
    voice.data = msg;
    pub_speaker.publish(voice);
    ros::Duration(2, 0).sleep();

    //pocketsphinx
    return 0;
}*/

/*void transform_zones()
{

	tf::TransformListener listener;
    tf::StampedTransform transform;

    //Transform 12 target zones
    for(int i=0; i<target_zones.size();i++){
    	//Obtaining destination point from string 
    	try{
          //listener.lookupTransform(target_zones.at(i), "/map", ros::Time(0), transform);
    		listener.lookupTransform(target_zones.at(i).data, "/map", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        tf_target_zones.at(i).pose.position.x = -transform.getOrigin().x();
    	tf_target_zones.at(i).pose.position.y = -transform.getOrigin().y();
    }
}*/

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;
    FestinoHRI::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);

    ros::Rate loop(30);

    //Speaker
    std::string voice;
    //Open Pose
    bool pointing_hand;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            //voice = "I am ready for the navigation challenge";
	            ros::Duration(2, 0).sleep();
	            FestinoHRI::enableSpeechRecognized(false);
	            voice = "I am ready for the navigation challenge";
	            FestinoHRI::say(voice, 5);
	    		state = SM_FIND_BAG;
	    		break;
	    	case SM_FIND_BAG:{
	    		std::cout << "State machine: SM_FIND_BAG" << std::endl;
	    		voice = "Please point at the object that you want me to carry";
	    		FestinoHRI::say(voice, 5);
				voice = "Are you already pointing at the object? tell me Robot yes or Robot no";
				FestinoHRI::say(voice, 5);

				//Enable speech recognition 
				FestinoHRI::enableSpeechRecognized(true);
	    		ros::Duration(2, 0).sleep();

	    		//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			pointing_hand = FestinoVision::PointingHand();
	    			if (pointing_hand){
		    			voice = "Please put the left bag on the platform";
		            	FestinoHRI::say(voice, 5);
	    			}
	    			else{
		    			voice = "Please put the right bag on the platform";
		            	FestinoHRI::say(voice, 5);
	    			}
	    			FestinoHRI::enableLegFinder(true);
	    			FestinoHRI::enableSpeechRecognized(false);
	    			state=SM_WAIT_FOR_BAG;
	    		}

	            //FestinoHRI::say(voice, 5);
	    		//Open pose 
	    		//pointing_hand = FestinoVision::PointingHand();
	    		//ros::Duration(2, 0).sleep();
	    		/*if (pointing_hand){
	    			voice = "Put the left bag on the platform";
	            	FestinoHRI::say(voice, 5);
	    		}
	    		else{
	    			voice = "Put the right bag on the platform";
	            	FestinoHRI::say(voice, 5);
	    		}*/

	    		//state = SM_FIND_PERSON;
	    		break;
	        }
	    	case SM_WAIT_FOR_BAG:
	    		std::cout << "State machine: SM_WAIT_FOR_BAG" << std::endl;	
	    		//Leg finder
	    		//Una vez encontrada que se presente Festino e indique al operador que señale la bolsa 
	    		/*voice = "Do you want me to carry a bag for you?, tell me Robot yes or Robot no";
	    		FestinoHRI::say(voice, 5);
	    		FestinoHRI::enableSpeechRecognized(true);
	    		ros::Duration(2, 0).sleep();

	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000))
                    std::cout << "yes" << std::endl;
                else 	
                	std::cout << "no" << std::endl;
	    		*/


	    		voice = "Have you already put the bag? tell me Robot yes or Robot no";
				FestinoHRI::say(voice, 5);


				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			FestinoHRI::enableSpeechRecognized(false);
					state = SM_FIND_PERSON;
	    		}

	    		
	            /*if(wait_conformation(msg)){
	            	state = SM_FIND_BAG;
	            }
	            else
	            {
	            	state = SM_FIND_PESON;
	            }*/
	            //state = SM_FINAL_STATE;
	    		break;
	    	case SM_FIND_PERSON:
				std::cout << "State machine: SM_FIND_PESON" << std::endl;	
    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not foun" << std::endl;
	    			FestinoHRI::enableHumanFollower(false);
	    			state = SM_FIND_PERSON;
	    		}
    			else{
	    			FestinoHRI::enableHumanFollower(true);
	    			state = SM_FINAL_STATE;
	    		}

	    		break;
	    	/*case SM_FIND_BAG:{
	    		std::cout << "State machine: SM_FIND_BAG" << std::endl;
	    		//Open pose 
	    		//Esperar a que el operador señale la bolsa
	    		//Buscar la pose de la bolsa señalada 
	    		state = SM_ASK_FOR_BAG;
	    		break;
	        }
	    	case SM_ASK_FOR_BAG:{
	    		std::cout << "State machine: SM_ASK_FOR_BAG" << std::endl;
            	//Ask the operator to put the bag on the platform
            	msg = "Please put the bag on the platform";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            //Waiting for the bag
	            ros::Duration(5, 0).sleep();

	            state = SM_INSTRUCTION;

	            //if(FestinoHRI::Says(msg,waiting_val)){
	            //	state = SM_INSTRUCTION;
	            //}
	            break;
	        }
	    	case SM_CONFIRM_BAG:{
	    		std::cout << "State machine: SM_CONFIRM_BAG" << std::endl;
	    		msg = "Is the bag already on the platform?, tell me Festino yes or Festino no";
	            if(wait_conformation(msg)){
	            	//Ask the operator to say follow when ready
	            	msg = "Please say follow me when ready";
	            	if(wait_conformation(msg)){
	            		state = SM_FOLLOW_OPERATOR;
	            	}
	            }
	            else
	            {
	            	state = SM_CONFIRM_BAG;
	            }
	        case SM_FOLLOW_OPERATOR:{
	        	std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;
            	//Esperar a que siga la señal de alto
            	msg = "Please say stop when we reach the car location";
            	if(!wait_conformation(msg)){
            		//Seguir al operador (human follow)
            		state = SM_FOLLOW_OPERATOR;
	            }
	            else
	            {
	            	msg = "Please say stop when we reach the car location";
	            	state = SM_BAG_DELIVERY_PLACE;
	            }
	            break;
	        }
	        case SM_BAG_DELIVERY_PLACE:{
            	//Decirle al operador que tome la bolsa
            	state = SM_WAIT_BAG_OFF;
	            break;
	        }
	        case  SM_WAIT_BAG_OFF:{
            	//Esperar a que el operador quite la bolsa
            	state = SM_FINAL_STATE;
	            break;
	        }*/
	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished test";
	            ros::Duration(30, 0).sleep();
	            FestinoHRI::enableLegFinder(false);
	    		FestinoHRI::enableHumanFollower(false);
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