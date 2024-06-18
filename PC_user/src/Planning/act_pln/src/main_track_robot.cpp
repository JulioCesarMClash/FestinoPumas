//State Machine for main track
#include<iostream>
#include <cmath>
#include <math.h>
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

//Biblioteca para tokenizar
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>	

//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"


//Se puede cambiar, agregar o eliminar los estados
enum SMState {
    SM_INIT,
	SM_WAIT_FOR_INSTRUCTION,
	SM_GO_TO,
	SM_FIND,
    SM_TAKE,
    SM_MOVE,
    SM_DROP,
    SM_ASK,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;

std_msgs::String target_zone;
geometry_msgs::PoseStamped tf_target_zone;
std::vector<geometry_msgs::PoseStamped> zones_path;

//String that storage instruction tokens
std::vector<std::string> tokens;

std::vector<std::string> real_refbox_names;
std_msgs::String new_zone;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

bool request = false;


void compute_coordinates(){
    if(tokens[4] == "entrance" || tokens[4] == "platform" ){
        tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + cos(stoi(tokens[3])*(M_PI/180));
        tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + sin(stoi(tokens[3])*(M_PI/180));                        
    }
    if(tokens[4] == "output"){
        tf_target_zone.pose.position.x = tf_target_zone.pose.position.x - cos(stoi(tokens[3])*(M_PI/180));
        tf_target_zone.pose.position.y = tf_target_zone.pose.position.y - sin(stoi(tokens[3])*(M_PI/180));
    }
}

void transform_zone()
{
	tf::TransformListener listener;
    tf::StampedTransform transform;

    //TF related stuff 
    std::cout << tokens[2] << std::endl;
    tf_target_zone.header.frame_id = "/map";
    tf_target_zone.pose.position.x = 0.0;
    tf_target_zone.pose.position.y = 0.0;
    tf_target_zone.pose.position.z = 0.0;
    tf_target_zone.pose.orientation.x = 0.0;
    tf_target_zone.pose.orientation.y = 0.0;
    tf_target_zone.pose.orientation.z = 0.0;
    tf_target_zone.pose.orientation.w = 0.0;

    std::cout << "entró al transform zones" << std::endl;

    try{
        std::cout << "entró al try" << std::endl;
        listener.waitForTransform(tokens.at(2), "/map", ros::Time(0), ros::Duration(1000.0));
        listener.lookupTransform(tokens.at(2), "/map", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    tf_target_zone.pose.position.x = -transform.getOrigin().x();
    tf_target_zone.pose.position.y = -transform.getOrigin().y();

    std::cout << "salió del try name:" << tokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;

}

void navigate_to_location(geometry_msgs::PoseStamped location)
{
    std::cout << "Navigate to location x:"<< location.pose.position.x << " y:" << location.pose.position.y << std::endl;
    if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x,60000)){
        if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x, 60000)){
         	std::cout << "Cannot move to " << std::endl;
                FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
}


void callback_instructions(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "Entré al callback" << msg->data.c_str() <<std::endl;	
    //Tokenize instruction string
    target_zone = *msg;
    tokens.clear();
    boost::algorithm::split(tokens, target_zone.data, boost::algorithm::is_any_of(" "));
    request = false;

    if(tokens[0] == "goto"){
        state = SM_GO_TO;
        return;
    }

    if(tokens[0] == "find"){
        state = SM_FIND;
        return;
    }

    if(tokens[0] == "take"){
        state = SM_TAKE;
        return;
    }

    if(tokens[0] == "move"){
        state = SM_MOVE;
        return;
    }

    if(tokens[0] == "drop"){
        state = SM_DROP;
        return;
    }

    if(tokens[0] == "ask"){
        state = SM_ASK;
        return;
    }
}


int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;
	
	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);

    //Subscribers and Publishers
    ros::Subscriber subInstructions = n.subscribe("/instruction_msg", 10, callback_instructions);
    ros::Publisher pubRequest       = n.advertise<std_msgs::String>("/request_instruction", 1000);
    ros::Publisher pub_rosnav_goal  = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000, true);
    ros::Rate loop(30);

    std::string voice;
    std_msgs::String request_string;
    request_string.data = "Ola khe ase";

    int cont = 0;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the main track challenge";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,5);
	    		state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_WAIT_FOR_INSTRUCTION:
	    		std::cout << "State machine: SM_WAIT_FOR_INSTRUCTION" << std::endl;	
	            voice = "Waiting for instruction";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,5);
                //Ask for instruction once
                if(!request){
                    pubRequest.publish(request_string);
                    request = true;
                    std::cout << "Ya mandé el request" << std::endl;	
                }

				//Waiting for instruction
	
	    		state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

	    	case SM_GO_TO:
	    		std::cout << "State machine: SM_GO_TO" << std::endl;
	            voice = "Navigating to destination point";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

                transform_zone();
                compute_coordinates();

                //navigate_to_location(tf_target_zone);
                pub_rosnav_goal.publish(tf_target_zone);
				ros::Duration(10, 0).sleep();

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

	    	case SM_FIND:
            	//Wait for finished navigation
	            std::cout << "State machine: SM_FIND" << std::endl;
	            voice = "Navigating to destination point";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
            
                

                state = SM_WAIT_FOR_INSTRUCTION;
	            break;
	        
			case SM_TAKE:
	    		//Navigate case
	    		std::cout << "State machine: SM_TAKE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_MOVE:
	    		//Navigate case
	    		std::cout << "State machine: SM_MOVE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_DROP:
	    		//Navigate case
	    		std::cout << "State machine: SM_DROP" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_ASK:
	    		//Navigate case
	    		std::cout << "State machine: SM_ASK" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;
		}
        ros::Duration(1, 0).sleep();
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}