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
#include "img_proc/Find_tag_Srv.h"


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_GO_ZONE,
	SM_NAV_FWD,
	SM_NAV_AROUND_OBST,
	SM_TAG_SEARCH,
	SM_GIRO,
	SM_TAG_DETECTED,
    SM_FINAL_STATE,
    SM_SCAN_SPACE
};

template <typename T>
void print_vector(const std::vector<T> & vec, std::string sep=" ")
{
    for(auto elem : vec)
    {
        std::cout<<elem<< sep;
    }
    std::cout<<std::endl;
}

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;
std::vector<std_msgs::String> target_zones;
geometry_msgs::PoseStamped det_mps;
std_msgs::String new_zone;
std_msgs::String mps_name;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool mps_flag;
std::vector<std_msgs::String> mps_names;
bool flag_names = false;
std_msgs::String mps_id;

//Logistics zones
std::vector<geometry_msgs::PoseStamped> zones_poses;
geometry_msgs::PoseStamped tf_zone;

//Zonas de prueba para el escaneo (se eligieron de forma que cubrieran gran parte del lab)
//M_Z53 //M_Z14 //M_Z22 //M_Z21

//Estas coordenadas se obtuvieron del archivo "challengeTracks_Zones.launch"
//Son las x de las zonas de escaneo respecto al origen de logistics
//std::vector<float> tf_x {-5.5, -1.5, 0.5, 3.5};
//Son las y de las zonas de escaneo respecto al origen de logistics
//std::vector<float> tf_y {2.5, 3.5, 4.5, 3.5}; 


//Son las coordenadas respecto al mapa que considera solo las zonas disponibles
//Estas coordenadas se obtuvieron contando los cuadritos en el rviz respecto al origen del mapa
//Son las x de las zonas de escaneo respecto al mapa
std::vector<float> tf_x {0.5, 4.5, 6.5, 9.5};
//Son las y de las zonas de escaneo respecto al mapa
std::vector<float> tf_y {-4.5, -3.5, -2.5, -3.5}; 

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
	if(mps_names.size() == 0){
		mps_names.push_back(mps_name);
	}
	else if(!(std::count(mps_names.begin(), mps_names.end(), mps_name))){
		mps_names.push_back(mps_name);
	}
	if(mps_names.size() == 4)
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

void transform_mps()
{
	if(mps_names.size() > 0){
		for(int i = 0; i < mps_names.size(); i++){
			ros::Duration(0.5, 0).sleep();
			std::cout << "\n" << mps_names[i].data << "\n" << std::endl;
			tf::TransformListener listener;
			tf::StampedTransform transform;

			//TF related stuff 
			det_mps.pose.position.x = 0.0;
			det_mps.pose.position.y = 0.0;
			det_mps.pose.position.z = 0.0;
			det_mps.pose.orientation.x = 0.0;
			det_mps.pose.orientation.y = 0.0;
			det_mps.pose.orientation.z = 0.0;
			det_mps.pose.orientation.w = 0.0;

			try{
				listener.waitForTransform(mps_names[i].data, "/camera_link", ros::Time(0), ros::Duration(1000.0));
				listener.lookupTransform(mps_names[i].data, "/camera_link", ros::Time(0), transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			det_mps.pose.position.x = -transform.getOrigin().x();
			det_mps.pose.position.y = -transform.getOrigin().y();
			det_mps.pose.position.z = -transform.getOrigin().z();
			
			std::cout << det_mps.pose.position << std::endl;
		}
	}
}

void navigate_to_location(geometry_msgs::PoseStamped location)
{
    std::cout << "Navigate to location x:"<< location.pose.position.x << " y:" << location.pose.position.y << std::endl;
    if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x,60000)){
		//La función espera a que llegue a la localidad
        if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x, 60000)){
         	std::cout << "Cannot move to " << std::endl;
                FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
}

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	bool tag_flag;
	bool giro =false;

	std::cout << "INITIALIZING EXPLORATION NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);

    //Subscribers and Publishers
    ros::Subscriber subRefbox 				= n.subscribe("/zones_refbox", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   	= n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_mps_flag     = n.subscribe("/aruco_det", 10, callback_mps_flag);
	ros::Subscriber sub_mps_name     = n.subscribe("/mps_name", 10, callback_mps_name);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);
    ros::ServiceClient client = n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");
    img_proc::Find_tag_Srv srv;

    ros::Rate loop(10);

    std::string voice;

    std::vector<std::string> mps_name;
    std::vector<geometry_msgs::PointStamped> mps_aruco;

	 //TF related stuff 
	 //Se tiene que a fuerza inicializar las poseStamped porque marca error si no se hace
	for(int i=0; i<tf_x.size(); i++){
    	tf_zone.header.frame_id = "/map";
	    tf_zone.pose.position.x = tf_x.at(i);
	    tf_zone.pose.position.y = tf_y.at(i);
	    tf_zone.pose.position.z = 0.0;
	    tf_zone.pose.orientation.x = 0.0;
	    tf_zone.pose.orientation.y = 0.0;
	    tf_zone.pose.orientation.z = 0.0;
	    tf_zone.pose.orientation.w = 0.0;
	    zones_poses.push_back(tf_zone);
    }

	//Contador que lleva el número de zonas que se han recorrido
	int cont = 0;

	//Contador que lleva el número de giros
	int cont_giro = 0;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:{
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the exploration challenge";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		state = SM_SCAN_SPACE;
	    		break;
			}

			case SM_GO_ZONE:{
				std::cout << "State machine: SM_GO_ZONE" << std::endl;	
				//Se navega a las zonas recorriendo el arreglo zones_poses
				//El contador es el índice que recorre el arreglo

				//Cuando se hayan recorrido las 4 zonas ya terminó
				if(cont < 4){
					// TestComment
					navigate_to_location(zones_poses.at(cont));
					ros::Duration(1, 0).sleep();
		            // TestComment
					std::cout << "Im in Zone  " << cont << std::endl;	
					state = SM_SCAN_SPACE;
				}
				else{
					std::cout << "All Zones visited" << std::endl;
					if(flag_names){
						std::cout << "I found all the Tags" << std::endl;
						state = SM_FINAL_STATE;
					}
					else{
						std::cout << "Still not all the tags" << std::endl;
						std::cout << "En esta vida hay limites, se hizo lo que se pudo" << std::endl;
						state = SM_FINAL_STATE;
					}
				}
				break;
			}

	    	case SM_SCAN_SPACE:{
	    		//Looking for Obstacles
	    		std::cout << "State machine: SM_SCAN_SPACE" << std::endl;
	            /*msg = "Turn arooound";
	            std::cout << msg << std::endl;
	            voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(2, 0).sleep();
	            sleep(1);*/
				
				ros::Duration(5, 0).sleep();
				std::cout << "Estoy escaneando zzz" << std::endl;
	            srv.request.is_find_tag_enabled = true;
				tag_flag = false;
				if (client.call(srv))
				{
					tag_flag = srv.response.success;
					if (tag_flag == true)
					{
						mps_name = srv.response.mps_name;
						mps_aruco = srv.response.point_stamped;
						print_vector(mps_names,",");
					}
					
					//mps_name = "Not Identified";
					/*if(tag_flag and mps_name != srv.response.mps_name)

				  if (client.call(srv))
				  {
				  	tag_flag = srv.response.success;
				  	//mps_name = srv.response.mps_name;
				  	mps_name = "Not Identified";
				    //if(tag_flag and mps_name != srv.response.mps_name)
					if(1)
					{
						std::cout << "Tag Found" << std::endl;
						state = SM_TAG_DETECTED;
						tag_flag = false;
					}
					else{
						std::cout << "Tag Not Found - Turn" << std::endl;
						state = SM_GIRO;
					}*/
				}
				else
				{
					ROS_ERROR("Failed to call service to search Tag");
					return 1;
				}
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(3, 0).sleep();*/

				// ----- If Tag Detected -> Send Information and discard as go_to_obstacle -----
				// ----- Else -> Continue to navigate -----
				
	            break;
	        }
			case SM_GIRO:{
				std::cout << "State machine: SM_GIRO" << std::endl;
				if(cont_giro < 3){
					std::cout << "Turn arooound" << std::endl;
					//Da un giro de 360 grados (2pi) para escanear todo
					//Le puse un ángulo de 6.2832 pero no funciona así

					//TestComment
					//FestinoNavigation::moveDistAngle(0.0, 1.2, 10000);
					// los 360 grados es demasiada vuelta (MitComment)
					//FestinoNavigation::moveDistAngle(0.0, 2.4, 10000);
					//TestComment
					tag_flag = false;
					cont_giro++;
					state = SM_SCAN_SPACE;
				}
				else{
					std::cout << "Navigating to New Zone" << std::endl;
					cont++;
					state = SM_GO_ZONE;
				}
			}
			case SM_TAG_DETECTED:{
	    		//Scan Tag and Send Information
	    		std::cout << "State machine: SM_TAG_DETECTED" << std::endl;	
	            voice =  "Looking for information";
	            std::cout << voice << std::endl;
	            tag_flag = false;
	            //std::cout << srv.response.mps_name << std::endl;
	            //std::cout << srv.response.point_stamped.point.x << std::endl;

				//voice = "This is the " + srv.response.mps_name;
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

				//transform_mps();
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
					cont++;
					state = SM_GIRO;
				}
	    		break;
			}
	    	case SM_FINAL_STATE:{
	    		//Finish
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
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