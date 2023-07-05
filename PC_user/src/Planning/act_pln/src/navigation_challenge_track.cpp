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

//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"


//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_WAIT_FOR_ZONES,
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
std::vector<geometry_msgs::PoseStamped> zones_path;
std::vector<std::string> tokens;
std::vector<std::string> real_refbox_names;
std_msgs::String new_zone;
std::string zone_name;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

geometry_msgs::PoseStamped location_test;

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
	tokens.erase(tokens.begin() + tokens.size());	
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
		std::cout << tokens[i] << std::endl;
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
    for(int i=0; i< tokens.size(); i++){
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

        

        tf_target_zones.at(i).pose.position.x = -transform.getOrigin().x();
    	tf_target_zones.at(i).pose.position.y = -transform.getOrigin().y();

std::cout << "salió del try name:" << tokens.at(i) << " tf x:" << tf_target_zones.at(i).pose.position.x << " y:" << tf_target_zones.at(i).pose.position.y << std::endl;

    	std::cout << "pasó las tfs" << std::endl;
    }

    std::cout << "salió del for" << std::endl;
}

void nearest_neighbour()
{
	//Inicialización de variables
	double min_dist;
	int min_indx;
	geometry_msgs::PoseStamped tf_nearest_zone;
	geometry_msgs::PoseStamped tf_zone;

	//Vector of euclidean distances
	std::vector<double> euc_dist;

	//Obtaining robot location
	geometry_msgs::PoseStamped tf_robot_pose;
	tf::TransformListener listener_rob;
    tf::StampedTransform transform_rob;

    try{
      listener_rob.waitForTransform("/map", "/base_link",  
                                   ros::Time(0), ros::Duration(1000.0));
      listener_rob.lookupTransform("/map", "/base_link",  
                                   ros::Time(0), transform_rob);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf_robot_pose.pose.position.x = transform_rob.getOrigin().x();
	tf_robot_pose.pose.position.y = transform_rob.getOrigin().y();

	std::cout << "La pose del robot es: " << tokens.at(min_indx) << std::endl;
	std::cout << "Coords x: " << tf_robot_pose.pose.position.x << " y:" << tf_robot_pose.pose.position.y << std::endl;

	//Mientras el tamaño del vector de zonas sea mayor a cero seguirá recorriendo
	while(tf_target_zones.size() > 0){

		//Inicializa la distancia mínima como infinito
		min_dist = std::numeric_limits<double>::infinity();

		//Calculating Euclidean distance to every zone
	    for(int i=0; i<tf_target_zones.size(); i++){
	    	tf_zone = tf_target_zones.at(i);

	    	double diff_x =  tf_robot_pose.pose.position.x - tf_zone.pose.position.x;
	    	double diff_y =  tf_robot_pose.pose.position.y - tf_zone.pose.position.y;

	    	double dist = pow(diff_x, 2) + pow(diff_y, 2);

			//Finding the min distance 
	    	if(dist < min_dist){
	    		min_dist = dist;
	    		tf_nearest_zone = tf_zone;
	    		min_indx = i;
	    	}
	    }

	    //Agregar la zona más cercana al vector que representa el camino a seguir
	    zones_path.push_back(tf_nearest_zone);

	    //DEBUGGING BORRAR DESPUES
	    std::cout << "La siguiente zona es: " << tokens.at(min_indx) << std::endl;
		std::cout << "Coords x: " << tf_nearest_zone.pose.position.x << " y:" << tf_nearest_zone.pose.position.y << std::endl;

	    //Quita del vector las zonas que ya son recorridas
	    //Delete location from zones vector
	    tf_target_zones.erase(tf_target_zones.begin() + min_indx);
		tokens.erase(tokens.begin() + min_indx);

	    //Ahora la nueva posición del robot es la zona más cercana
		tf_robot_pose.pose.position.x = tf_nearest_zone.pose.position.x;
		tf_robot_pose.pose.position.y = tf_nearest_zone.pose.position.y;

	}

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

void define_zone(geometry_msgs::PoseStamped location)
{
	if(location.pose.position.x  > -1 && location.pose.position.x <= 0 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "M_Z11";
	}
	else if(location.pose.position.x  > -1 && location.pose.position.x <= 0 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "M_Z12";
	}
	else if(location.pose.position.x  > -1 && location.pose.position.x <= 0 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "M_Z13";
	}
	else if(location.pose.position.x  > -1 && location.pose.position.x <= 0 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "M_Z14";
	}
	else if(location.pose.position.x  > -1 && location.pose.position.x <= 0 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "M_Z15";
	}
	else if(location.pose.position.x  > -2 && location.pose.position.x <= -1 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "M_Z21";
	}
	else if(location.pose.position.x  > -2 && location.pose.position.x <= -1 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "M_Z22";
	}
	else if(location.pose.position.x  > -2 && location.pose.position.x <= -1 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "M_Z23";
	}
	else if(location.pose.position.x  > -2 && location.pose.position.x <= -1 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "M_Z24";
	}
	else if(location.pose.position.x  > -2 && location.pose.position.x <= -1 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "M_Z25";
	}
	else if(location.pose.position.x  > -3 && location.pose.position.x <= -2 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "M_Z31";
	}
	else if(location.pose.position.x  > -3 && location.pose.position.x <= -2 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "M_Z32";
	}
	else if(location.pose.position.x  > -3 && location.pose.position.x <= -2 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "M_Z33";
	}
	else if(location.pose.position.x  > -3 && location.pose.position.x <= -2 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "M_Z34";
	}
	else if(location.pose.position.x  > -3 && location.pose.position.x <= -2 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "M_Z35";
	}
	else if(location.pose.position.x  > -4 && location.pose.position.x <= -3 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "M_Z41";
	}
	else if(location.pose.position.x  > -4 && location.pose.position.x <= -3 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "M_Z42";
	}
	else if(location.pose.position.x  > -4 && location.pose.position.x <= -3 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "M_Z43";
	}
	else if(location.pose.position.x  > -4 && location.pose.position.x <= -3 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "M_Z44";
	}
	else if(location.pose.position.x  > -4 && location.pose.position.x <= -3 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "M_Z45";
	}
	else if(location.pose.position.x  > -5 && location.pose.position.x <= -4 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "M_Z51";
	}
	else if(location.pose.position.x  > -5 && location.pose.position.x <= -4 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "M_Z52";
	}
	else if(location.pose.position.x  > -5 && location.pose.position.x <= -4 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "M_Z53";
	}
	else if(location.pose.position.x  > -5 && location.pose.position.x <= -4 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "M_Z54";
	}
	else if(location.pose.position.x  > -5 && location.pose.position.x <= -4 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "M_Z55";
	}
	else{
		zone_name = "invalid";
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
    ros::Subscriber subRefbox = n.subscribe("/zone_msg", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);

    ros::Rate loop(30);

    std::string voice;

    int cont = 0;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the navigation challenge";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,5);
			
				location_test.header.frame_id = "/map";
				location_test.pose.position.x = -3.7;
				location_test.pose.position.y = 2.6;
				
				define_zone(location_test);
				std::cout << "La zona es: " << zone_name << std::endl;

	    		state = SM_WAIT_FOR_ZONES;
	    		break;

	    	case SM_WAIT_FOR_ZONES:
	    		//Wating for zone case
	    		std::cout << "State machine: SM_WAIT_FOR_ZONES" << std::endl;
	            voice = "Wating for target zones";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	            sleep(1);

	            //Waiting for 12 zones
	    		if(flag_zones == false){
	    			state = SM_WAIT_FOR_ZONES;	
	    		}	
	    		else{
	    			transform_zones();
	    			nearest_neighbour();
	    			state = SM_NAV_NEAREST_ZONE;	
	    		}
	    		break;
	    	case SM_NAV_NEAREST_ZONE:{
            	//Wait for finished navigation
	            std::cout << "State machine: SM_NAV_NEAREST_ZONE" << std::endl;
	            voice = "Navigating to destination point";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

	            navigate_to_location(zones_path.at(cont));
				ros::Duration(6, 0).sleep();
	            cont++;

				if(cont == 12){
					state = SM_FINAL_STATE;
				}

	            break;
	        }
	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		success = true;
	    		fail = true;
	    		break;
		}
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}