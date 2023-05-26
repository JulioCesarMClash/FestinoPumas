//Explore the Field
//Report the position and orientation of the MPSs
#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"
#include <algorithm>

//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_NAV_FWD,
	SM_NAV_AROUND_OBST,
	SM_TAG_SEARCH,
	SM_TAG_DETECTED,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;
std::vector<std_msgs::String> target_zones;
std::vector<geometry_msgs::PoseStamped> tf_target_zones;
std_msgs::String new_zone;
std_msgs::String mps_name;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool mps_flag;
std::vector<std_msgs::String> zones_names;
bool flag_names = false;
std::string zone;

void callback_refbox_zones(const std_msgs::String::ConstPtr& msg)
{
    new_zone = *msg;
    target_zones.push_back(new_zone);

    if(target_zones.size() == 12){
        flag_zones = true;
    }
}

//Arreglo con los nombres de las estaciones
void callback_mps_name(const std_msgs::String::ConstPtr& msg){
    mps_name = *msg;
	if(zones_names.size() == 0){
		zones_names.push_back(mps_name);
	}
	else if(!(std::count(zones_names.begin(), zones_names.end(), mps_name))){
		zones_names.push_back(mps_name);
	}
	if(zones_names.size() == 4)
	{
		flag_names = true;
	}
}

//Encontro un TAG
void callback_mps_flag(const std_msgs::Bool::ConstPtr& msg){
    mps_flag = msg->data;
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}

void transform_zones(std::string zone)
{
	tf::TransformListener listener;
    tf::StampedTransform transform;

    //TF related stuff 
    tf_target_zone.header.frame_id = "/map";
	tf_target_zone.pose.position.x = 0.0;
	tf_target_zone.pose.position.y = 0.0;
	tf_target_zone.pose.position.z = 0.0;
	tf_target_zone.pose.orientation.x = 0.0;
	tf_target_zone.pose.orientation.y = 0.0;
	tf_target_zone.pose.orientation.z = 0.0;
	tf_target_zone.pose.orientation.w = 0.0;

   	try{
   		listener.lookupTransform(zone, "/map", ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf_target_zones.at(i).pose.position.x = -transform.getOrigin().x();
    tf_target_zones.at(i).pose.position.y = -transform.getOrigin().y();
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
    ros::Subscriber subRefbox 				= n.subscribe("/zones_refbox", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   	= n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_mps_flag     = n.subscribe("/aruco_det", 10, callback_mps_flag);
	ros::Subscriber sub_mps_name     = n.subscribe("/mps_name", 10, callback_mps_name);
	ros::Publisher pub_speaker = n.advertise<std_msgs::String>("/speak", 1000, latch = true);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);

    ros::Rate loop(30);

    std_msgs::String voice;
    std::string msg;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:{
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            msg = "I am ready for the exploration challenge";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	    		state = SM_NAV_FWD;
	    		break;
			}

	    	case SM_NAV_FWD:{
	    		//Looking for Obstacles
	    		std::cout << "State machine: SM_NAV_FWD" << std::endl;
	            msg = "Looking for Obstacles";
	            std::cout << msg << std::endl;
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	            sleep(1);*/
				// ----- If obstacle found -> Go around it -----
				// ----- Else -> Go forward -----
				state = SM_NAV_AROUND_OBST;
	    		break;
			}

	    	case SM_NAV_AROUND_OBST:{
	    		//Navigate around the obstacle
	    		std::cout << "State machine: SM_NAV_AROUND_OBST" << std::endl;
	            msg = "Im going to navigate aroudn the station";
	            std::cout << msg << std::endl;
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();*/

				// ----- Go around -----
				// ----- Align -----
				// ----- Look for Tag -----
				// ----- OR Continue to Navigate -----
	            
	    		state = SM_TAG_SEARCH;
	    		break;
	        }

	    	case SM_TAG_SEARCH:{
            	//Look for Tag
	            std::cout << "State machine: SM_TAG_SEARCH" << std::endl;
	            msg = "Looking or Tag";
	            std::cout << msg << std::endl;
				if(mps_flag)
				{
					std::cout << "Tag Found" << std::endl;
					state = SM_TAG_DETECTED;
				}
				else
				{
					std::cout << "Waiting for Tag" << std::endl;
					state = SM_NAV_FWD;
				}
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(3, 0).sleep();*/

				// ----- If Tag Detected -> Send Information and discard as go_to_obstacle -----
				// ----- Else -> Continue to navigate -----
				
	            break;
	        }

			case SM_TAG_DETECTED:{
	    		//Scan Tag and Send Information
	    		std::cout << "State machine: SM_TAG_DETECTED" << std::endl;	
	            msg =  "I have found a Tag";
	            std::cout << msg << std::endl;
				std::cout << "Sending information" << std::endl;

	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();*/
	    		// ----- Send Information and discard as go_to_obstacle -----
				// ----- If Tags Detected >= 4 -> Fin -----
				// ----- Else -> Continue to navigate -----
				if(flag_names){
					std::cout << "I found all the Tags" << std::endl;
					state = SM_FINAL_STATE;
				}
				else{
					std::cout << "Still not all the tags" << std::endl;
					state = SM_NAV_FWD;
				}
	    		break;
			}

	    	case SM_FINAL_STATE:{
	    		//Finish
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            msg =  "I have finished test";
	            std::cout << msg << std::endl;
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();*/
	    		success = true;
	    		fail = true;
	    		break;
			}
		}
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}