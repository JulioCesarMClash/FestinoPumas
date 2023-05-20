//Drive to 12 randomly generated target zones.
//Refbox sends all 12 zones at once.
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

//Biblioteca para tokenizar
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>


//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_WAIT_FOR_ZONES,
	SM_CALC_EU_DIST,
	SM_NAV_NEAREST_ZONE,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;
//std::vector<std_msgs::String> target_zones;

//La funcion que hace la tokenizada no acepta std_msgs
std_msgs::String target_zones;

//Entonces hay que usar un simple std::string
//std::string target_zones;

std::vector<geometry_msgs::PoseStamped> tf_target_zones;
geometry_msgs::PoseStamped tf_target_zone;
std::vector<std::string> tokens;
std_msgs::String new_zone;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

//Callback para recibir las 12 zonas una a una
/*void callback_refbox_zones(const std_msgs::String::ConstPtr& msg)
{
    new_zone = *msg;
    target_zones.push_back(new_zone);

    if(target_zones.size() == 12){
        flag_zones = true;
    }
}*/

//Callback para recibir las 12 zonas de golpe
void callback_refbox_zones(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "entró al callback " << *msg << std::endl;
    target_zones = *msg;
    tokens.clear();
    boost::algorithm::split(tokens, target_zones.data, boost::algorithm::is_any_of(" "));
    //std::cout << tokens.at(0) << std::endl;	
    flag_zones = true;
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}

void transform_zones()
{

	tf::TransformListener listener;
    tf::StampedTransform transform;


    //TF related stuff 
	for(int i=0; i<tokens.size(); i++){
    	tf_target_zone.header.frame_id = "/map";
	    tf_target_zone.pose.position.x = 0.0;
	    tf_target_zone.pose.position.y = 0.0;
	    tf_target_zone.pose.position.z = 0.0;
	    tf_target_zone.pose.orientation.x = 0.0;
	    tf_target_zone.pose.orientation.y = 0.0;
	    tf_target_zone.pose.orientation.z = 0.0;
	    tf_target_zone.pose.orientation.w = 0.0;
	    tf_target_zones.push_back(tf_target_zone);
    }

    std::cout << "entró al transform zones" << std::endl;

    //Transform 12 target zones
    for(int i=0; i< tokens.size() -1; i++){
    	//Obtaining destination point from string 
    	try{
    		std::cout << "entró al try" << std::endl;
          //listener.lookupTransform(target_zones.at(i), "/map", ros::Time(0), transform);
    		listener.waitForTransform(tokens.at(i), "/map", ros::Time(0), ros::Duration(1000.0));
    		listener.lookupTransform(tokens.at(i), "/map", ros::Time(0), transform);
    		std::cout << tokens.at(i) << std::endl;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        std::cout << "salió del try" << std::endl;

        tf_target_zones.at(i).pose.position.x = -transform.getOrigin().x();
    	tf_target_zones.at(i).pose.position.y = -transform.getOrigin().y();

    	std::cout << "pasó las tfs" << std::endl;
    }
    std::cout << "salió del for" << std::endl;
}

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

    //Subscribers and Publishers
    ros::Subscriber subRefbox = n.subscribe("/zone_msg", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Publisher pub_speaker = n.advertise<std_msgs::String>("/speak", 1000, latch = true);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);

    ros::Rate loop(30);

    std_msgs::String voice;
    std::string msg;

    int min_indx;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            msg = "I am ready for the navigation challenge";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	    		state = SM_WAIT_FOR_ZONES;
	    		break;

	    	case SM_WAIT_FOR_ZONES:
	    		//Wating for zone case
	    		std::cout << "State machine: SM_WAIT_FOR_ZONES" << std::endl;
	            msg = "Wating for target zones";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	            sleep(1);

	            //Waiting for 12 zones
	    		if(flag_zones == false){
	    			state = SM_WAIT_FOR_ZONES;	
	    		}	
	    		else{
	    			transform_zones();
	    			state = SM_CALC_EU_DIST;	
	    		}
	    		break;

	    	case SM_CALC_EU_DIST:{
	    		//Computing euclidean distance case
	    		std::cout << "State machine: SM_CALC_EU_DIST" << std::endl;
	            msg = "Computing euclidean distance";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();

	            //Vector of euclidean distances
				std::vector<double> euc_dist;

				//Obtaining robot location
				geometry_msgs::PoseStamped tf_robot_pose;
				tf::TransformListener listener_rob;
			    tf::StampedTransform transform_rob;
 
			    try{
			      listener_rob.waitForTransform("/base_link", "/map",  
			                                   ros::Time(0), ros::Duration(1000.0));
		          listener_rob.lookupTransform("/base_link", "/map",  
			                                   ros::Time(0), transform_rob);
		        }
		        catch (tf::TransformException ex){
		          ROS_ERROR("%s",ex.what());
		          ros::Duration(1.0).sleep();
		        }

		        tf_robot_pose.pose.position.x = -transform_rob.getOrigin().x();
		    	tf_robot_pose.pose.position.y = -transform_rob.getOrigin().y();
			   

	            //Calculating Euclidean distance to every zone
	            for(int i=0; i<tf_target_zones.size(); i++){
	            	double diff_x =  tf_robot_pose.pose.position.x - tf_target_zones.at(i).pose.position.x;
	            	double diff_y =  tf_robot_pose.pose.position.y - tf_target_zones.at(i).pose.position.y;

	            	double dist = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

	            	euc_dist.push_back(dist);
	            }

	            //Finding the min distance 
	            auto min_dist = std::min_element(euc_dist.begin(), euc_dist.end());
    			min_indx = std::distance(euc_dist.begin(), min_dist);

    			ros::Duration(2, 0).sleep();
                
                pub_goal.publish(tf_target_zones.at(min_indx));
                std::cout << tokens[min_indx] << std::endl;

	    		state = SM_NAV_NEAREST_ZONE;
	    		break;
	        }
	    	case SM_NAV_NEAREST_ZONE:{
            	//Wait for finished navigation
	            std::cout << "State machine: SM_NAV_NEAREST_ZONE" << std::endl;
	            msg = "Navigating to destination point";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(3, 0).sleep();

//TODO Cambiar por Festino::Tools
	            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1){
	                msg = "Goal location reached";
	                std::cout << msg << std::endl;
	                voice.data = msg;
	                pub_speaker.publish(voice);

	               	//Stay at zone for 5 seconds
	               	ros::Duration(5, 0).sleep();

	            	//Send location to refbox
	            	std::cout << "Yo ya estoy" << std::endl;

	            	//Delete location from zones vector
	            	tf_target_zones.erase(tf_target_zones.begin() + min_indx);

	            	//If all zones have been visited then go to final state 
	            	//otherwise calculate the euclidean distance again from the new zone
		            if(tf_target_zones.size() == 0){
		            	state = SM_FINAL_STATE;
		            }
		            else{
						state = SM_CALC_EU_DIST;
		            }
	            }
	            break;
	        }
	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            msg =  "I have finished test";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	    		success = true;
	    		fail = true;
	    		break;
		}
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}