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

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_FIND_PERSON,
	SM_WAIT_FOR_ZONES,
	SM_CALC_EU_DIST,
	SM_NAV_NEAREST_ZONE,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;
std::vector<std_msgs::String> target_zones;
std::vector<geometry_msgs::PoseStamped> tf_target_zones;
std_msgs::String new_zone;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;


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
}

/*void wait_conformation(msg)
{
    std::cout << msg << std::endl;
    voice.data = msg;
    pub_speaker.publish(voice);
    ros::Duration(2, 0).sleep();

    //pocketsphinx
    return 0;
}*/

void transform_zones()
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
}

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;
    FestinoHRI::setNodeHandle(&n);

    //Subscribers and Publishers
    ros::Subscriber subRefbox = n.subscribe("/zones_refbox", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    //ros::Publisher pub_speaker = n.advertise<std_msgs::String>("/speak", 1000, latch = true);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);

    ros::Rate loop(30);

    std::string voice;

    int min_indx;

    //TF related stuff 
	for(int i=0; i<target_zones.size(); i++){
    	tf_target_zones.at(i).header.frame_id = "/map";
	    tf_target_zones.at(i).pose.position.x = 0.0;
	    tf_target_zones.at(i).pose.position.y = 0.0;
	    tf_target_zones.at(i).pose.position.z = 0.0;
	    tf_target_zones.at(i).pose.orientation.x = 0.0;
	    tf_target_zones.at(i).pose.orientation.y = 0.0;
	    tf_target_zones.at(i).pose.orientation.z = 0.0;
	    tf_target_zones.at(i).pose.orientation.w = 0.0;
    }

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            //voice = "I am ready for the navigation challenge";
	            ros::Duration(2, 0).sleep();
	            voice = "I am ready for the navigation challenge";
	            FestinoHRI::say(voice, 5);
	    		state = SM_FIND_PERSON;
	    		break;
	    	case SM_FIND_PERSON:
	    		std::cout << "State machine: SM_FIND_PESON" << std::endl;	
	    		//Leg finder
	    		//Una vez encontrada que se presente Festino e indique al operador que señale la bolsa 
	    		voice = "Do you want me to carry a bag for you?, tell me Festino yes or Festino no";
	    		FestinoHRI::enableLegFinder(true);

	    		if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not foun" << std::endl;
	    			FestinoHRI::enableHumanFollower(false);
	    			state = SM_FIND_PERSON;
	    		}
	    		else{
	    			FestinoHRI::enableHumanFollower(true);
	    			state = SM_FINAL_STATE;
	    		}
	    		
	            /*if(wait_conformation(msg)){
	            	state = SM_FIND_BAG;
	            }
	            else
	            {
	            	state = SM_FIND_PESON;
	            }*/
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