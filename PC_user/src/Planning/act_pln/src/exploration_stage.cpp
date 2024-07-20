//De la arena completa solo se usa una mitad (Magenta o Cyan) y de esa solo se agarra 5x5.
//La arena completa mide 14 x 8. 


//¿Qué pasa si una máquina está muy cerca de un punto de escaneo? El kinect no verá el Aruco.
//¿Qué pasa si hay una máquina en un punto de escaneo?  Se queda en el punto más cercano y escanea desde ahí
//¿Qué pasa si un aruco está muy lejos para que la nube de puntos lo detecte? pos fracasa la detección :c

//Explore the Field
//Report the name, zone and orientation of the MPS
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
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

//#include <festino_arm_moveit_demos/srv_arm.h>
#include <img_proc/MPS_Detector.h>


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"


#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

//Biblioteca para tokenizar
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>	

using namespace std;

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_FIRSTMAPPING,
	SM_NAV_PIPS,
	SM_TURN_AROUND,
	SM_TAG_DETECTED,
    SM_FINAL_STATE
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

std_msgs::String mps_data2send;
//std_msgs::String mps_name2send;
std_msgs::String mps_name2send;

bool fail = false;
bool success = false;
SMState state = SM_INIT;
bool flag_zones = false;
std::vector<std_msgs::String> target_zones;
geometry_msgs::PoseStamped det_mps;
geometry_msgs::PoseStamped robot_curr_pos;
geometry_msgs::PoseStamped robot_next_pos;
geometry_msgs::PoseStamped robot2mps;
geometry_msgs::PoseStamped robot_last_pos;
geometry_msgs::PoseStamped robot_pos;
geometry_msgs::PoseStamped goal_tosend;

std_msgs::Bool time_over;

std_msgs::String new_zone;
std_msgs::String mps_name_anterior;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool mps_flag;
std::vector<std_msgs::String> mps_names;
bool flag_names = false;
std_msgs::String mps_id;
geometry_msgs::PoseStamped Log_origin;

bool nav_flag = false;
bool nav_success = false;

//look for tag function
bool tag_flag = false;
std::vector<std::string> mps_name;
std::vector<geometry_msgs::PointStamped> mps_PointStamped;
std::vector<int> cam_reg;
//look for tag function

//Logistics zones
// Anterior
std::vector<geometry_msgs::PoseStamped> zones_poses;
geometry_msgs::PoseStamped tf_zone;


// PIPs
std::vector<geometry_msgs::PoseStamped> pips_poses;
geometry_msgs::PoseStamped tf_pips_m;
geometry_msgs::PoseStamped tf_pips_c;
std_msgs::String pips_as_zones[4];

// PIIs
// Los piis no pueden ser as zones, son cords
std::vector<geometry_msgs::PoseStamped> piis_poses;
geometry_msgs::PoseStamped tf_piis;

sensor_msgs::LaserScan laserScan;
bool flag_door = true;

geometry_msgs::Twist tw_tomap;

bool act = false;

bool contains_str(std::string str, std::string word){
	std::vector<std::string> tokens;
	boost::algorithm::split(tokens, str, boost::algorithm::is_any_of("-"));
	for(int i=0;i<tokens.size();i++){
		if(tokens[i]==word){return true;} 
	}

	return false;
}

void nav_zone_to_cords(ros::NodeHandle n, std::string zone, geometry_msgs::PoseStamped)
{
    std::cout << "Zone " << zone << " recieved, coords to navigate: " << std::endl;
    tf::TransformListener listener;
	tf::StampedTransform transform;

	geometry_msgs::PoseStamped robot_pos;
    robot_pos.header.frame_id = "/map";
    robot_pos.pose.position.x = 0.0;
	robot_pos.pose.position.y = 0.0;
	robot_pos.pose.position.z = 0.0;
	robot_pos.pose.orientation.x = 0.0;
	robot_pos.pose.orientation.y = 0.0;
	robot_pos.pose.orientation.z = 0.0;
	robot_pos.pose.orientation.w = 0.0;

	try{
		ros::Duration(1.0).sleep();
		listener.waitForTransform("/map", zone, ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/map", zone, ros::Time(0), transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	goal_tosend.pose.position.x = transform.getOrigin().x();
	goal_tosend.pose.position.y = transform.getOrigin().y();
	goal_tosend.pose.position.z = transform.getOrigin().z();
	goal_tosend.pose.orientation.x = transform.getRotation().x();
	goal_tosend.pose.orientation.y = transform.getRotation().y();
	goal_tosend.pose.orientation.z = transform.getRotation().z();
	goal_tosend.pose.orientation.w = transform.getRotation().w();

	std::cout << goal_tosend.pose << std::endl;
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
    laserScan = *msg;
    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    //std::cout<< "lecturas Hokuyo" << range <<std::endl;

    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    for(int i=0; i<20;i++){
    	laserScan.ranges[i] = 10.0;
    }
    for(int i=1060; i<1081;i++){
    	laserScan.ranges[i] = 10.0;
    }

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4)
        { 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.50)
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

							//---------TESIS---------//
//Se toman las coordenadas de acuerdo con los  propuestos
//en Mittesis: 8 PIPrincipales y 11 PIIntermedios:

//Agrego el punto de salida en cada mitad como primer elemento del arreglo


std::vector<float> x_pips;
std::vector<float> y_pips;
//MAGENTA

//HOME: (-4.5,0.5)

//PIPs:    M_Z72,      M_Z18,      M_Z78,    M_Z12
//PIPs: (-6.5,1.5), (-0.5,7.5), (-6.5,7.5), (-0.5,1.5)

//X respecto al mapa de logistics
//std::vector<float> x_pips_m {-4.5, -6.5, -0.5, -6.5, -0.5};
std::vector<float> x_pips_m {-4.5, -1.5, -6.5, -0.5};
//Y respecto al mapa
//std::vector<float> y_pips_m {0.5, 1.5, 7.5, 7.5, 1.5}; 
std::vector<float> y_pips_m {1.5, 2.5, 7.5, 1.5}; 

//X respecto a la posición inicial del robot M_Z51                       ->Cyan
//std::vector<float> x_pips_m {1.0,1.0,1.0,2.0,3.0,4.0,5.0,5.0,5.0,4.0,3.0,1.0,2.0,3.0,4.0,5.0,5.0,5.0,4.0,3.0,2.0};
//Y respecto al mapa                                                             ->Cyan
//std::vector<float> y_pips_m {-1.0,-2.0,-3.0,-3.0,-3.0,-3.0,-2.0,-1.0,0.0,1.0,1.0,-7.0,-7.0,-7.0,-7.0,-7.0,-8.0,-9.0,-9.0,9.0,9.0}; 

//PIIs: (-5,3), (-2,6), (-3.5,7.5), (-5,6), (-2,3)

//X respecto al mapa
std::vector<float> x_piis_m {0, -5, -2, -3.5, -5, -2, 0};
//Y respecto al mapa
std::vector<float> y_piis_m {0, 3, 6, 7.5, 6, 3, 4.5}; 

//CIAN

//HOME: (4.5,0.5)

//PIPs:    C_Z18,     C_Z72,    C_Z78,    C_Z12
//PIPs: (0.5,7.5), (6.5,1.5), (6.5,7.5), (0.5,1.5)

//X respecto al mapa de logistics
std::vector<float> x_pips_c {0.0,5.0, 6.5, 6.5, 0.5};
//Y respecto al mapa
std::vector<float> y_pips_c {0.0,5.0, 1.5, 7.5, 1.5}; 

//X respecto a la posición inicial del robot M_Z51
//std::vector<float> x_pips_c {1.0,  2.0, 3.0, 4.0, 5.0, 5.0, 5.0, 4.0,3.0,2.0};
//Y respecto al mapa
//std::vector<float> y_pips_c {-7.0,-7.0,-7.0,-7.0,-7.0,-8.0,-9.0,-9.0,9.0,9.0};

//PIIs: (0,4.5), (2,6), (5,3), (6.5,4.5), (5,6), (2,3)
//Estoy tomando el PII 41 para meterlo en la sección de los CYAN
//Puse el n_piis para que lo incluyera

//X respecto al mapa
std::vector<float> x_piis_c {0, 0, 2, 5, 6.5, 5, 2};
//Y respecto al mapa
std::vector<float> y_piis_c {0, 4.5, 6, 3, 4.5, 6, 3}; 


// El que estaba antes:
//X respecto al mapa
std::vector<float> tf_x {0, -6.5, -1.5, -1.5, -3.5, -4.5, -4.5, -0.5, -0.5};
//Y respecto al mapa
std::vector<float> tf_y {0, 1.5, 1.5, 3.5, 3.5, 4.5, 4.5, 1.5, 0.5, 4.5}; 

void field_color_coords(std::string color){
	if(color == "CYAN"){
		std::cout << "\n Definiendo coordenadas y zonas para CYAN \n" << std::endl;
		pips_as_zones[0].data = "/C_Z42";
		pips_as_zones[1].data = "/C_Z22";
		pips_as_zones[2].data = "/C_Z24";
		pips_as_zones[3].data = "/C_Z44";

		x_pips = x_pips_c;
	}
	else{
		std::cout << "\n Definiendo coordenadas y zonas para MAGENTA \n" << std::endl;
		pips_as_zones[0].data = "/M_Z42";
		pips_as_zones[1].data = "/M_Z22";
		pips_as_zones[2].data = "/M_Z24";
		pips_as_zones[3].data = "/M_Z44";
		x_pips = x_pips_m;
	}
}	

//Arreglo con los nombres de las estaciones
void callback_mps_name(const std_msgs::String::ConstPtr& msg){
    mps_name_anterior = *msg;
	if(mps_names.size() == 0){
		mps_names.push_back(mps_name_anterior);
	}
	else if(!(std::count(mps_names.begin(), mps_names.end(), mps_name_anterior))){
		mps_names.push_back(mps_name_anterior);
	}
	if(mps_names.size() == 4)
	{
		flag_names = true;
	}
}


void callback_time_over(const std_msgs::Bool::ConstPtr& msg)
{
    time_over = *msg;
}

/*void callback_mps_name(const std_msgs::String::ConstPtr& msg){
    mps_name2send = *msg;
}*/

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
				listener.waitForTransform("/camera_link", mps_names[i].data,  ros::Time(0), ros::Duration(1000.0));
				listener.lookupTransform("/camera_link", mps_names[i].data, ros::Time(0), transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			det_mps.pose.position.x = transform.getOrigin().x();
			det_mps.pose.position.y = transform.getOrigin().y();
			det_mps.pose.position.z = transform.getOrigin().z();
			
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
			nav_flag = false;
         	std::cout << "Cannot move to location" << std::endl;
                //FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
}

//Estas coordenadas son M_Z32, ahi sabemos que no habra maquina
std::vector<float> arreglo_x = {-2.5, 0.0, 0.0, 0.0, 0.0};
std::vector<float> arreglo_y = {1.5, 0.0, 0.0, 0.0, 0.0};

//Los angulos de este arreglo estan en radianes, en grados son: 0, 270, 180, 90
//std::vector<float> arreglo_alfa = {0, 4.71239, 3.14159, 1.5708};

//Los angulos de este arreglo estan en radianes, en grados son: 45, 90, 135, 180, 225, 270, 315, 0
//Avanza de 45 en 45 grados
std::vector<float> arreglo_alfa = {0.7853, 1.5708, 2.356, 3.14159, 3.9269, 4.71239, 5.4977, 0};

//Nuevas coordeandas del aruco respecto al mapa
float aruco_x;
float aruco_y;
std::string mps_name_contains;

float deg_rob_mps;

//Paso en el que se encuentre el robot para saber su posicion
int step = 0;

//Contador que lleva el número de giros
int cont_giro = 0;

//Contador que lleva el número de zonas que se han recorrido
int cont = 0;

geometry_msgs::PoseStamped location_map;

void transform_aruco_map(geometry_msgs::PoseStamped location)
{
	location_map.pose.position.x = -cos(arreglo_alfa.at(cont_giro))*location.pose.position.x + sin(arreglo_alfa.at(cont_giro))*location.pose.position.y + arreglo_x.at(cont);
	location_map.pose.position.y = sin(arreglo_alfa.at(cont_giro))*location.pose.position.x + cos(arreglo_alfa.at(cont_giro))*location.pose.position.y + arreglo_y.at(cont);
	std::cout << location_map.pose.position.x << location_map.pose.position.y << std::endl;

}

std::string zone_name;
std::string zone_name_obtained;
bool m_is_cyan = true;
//TODO CHANGE TO FALSE
bool m_team_set = true;

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
	/*else{
		zone_name = "invalid";
	}*/
	//CYAN
	if(location.pose.position.x  > 4 && location.pose.position.x <= 5 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "C_Z51";
	}
	else if(location.pose.position.x  > 4 && location.pose.position.x <= 5 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "C_Z52";
	}
	else if(location.pose.position.x  > 4 && location.pose.position.x <= 5 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "C_Z53";
	}
	else if(location.pose.position.x  > 4 && location.pose.position.x <= 5 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "C_Z54";
	}
	else if(location.pose.position.x  > 4 && location.pose.position.x <= 5 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "C_Z55";
	}
	else if(location.pose.position.x  > 3 && location.pose.position.x <= 4 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "C_Z41";
	}
	else if(location.pose.position.x  > 3 && location.pose.position.x <= 4 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "C_Z42";
	}
	else if(location.pose.position.x  > 3 && location.pose.position.x <= 4 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "C_Z43";
	}
	else if(location.pose.position.x  > 3 && location.pose.position.x <= 4 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "C_Z44";
	}
	else if(location.pose.position.x  > 3 && location.pose.position.x <= 4 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "C_Z45";
	}
	else if(location.pose.position.x  > 2 && location.pose.position.x <= 3 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "C_Z31";
	}
	else if(location.pose.position.x  > 2 && location.pose.position.x <= 3 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "C_Z32";
	}
	else if(location.pose.position.x  > 2 && location.pose.position.x <= 3 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "C_Z33";
	}
	else if(location.pose.position.x  > 2 && location.pose.position.x <= 3 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "C_Z34";
	}
	else if(location.pose.position.x  > 2 && location.pose.position.x <= 3 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "C_Z35";
	}
	else if(location.pose.position.x  > 1 && location.pose.position.x <= 2 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "C_Z21";
	}
	else if(location.pose.position.x  > 1 && location.pose.position.x <= 2 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "C_Z22";
	}
	else if(location.pose.position.x  > 1 && location.pose.position.x <= 2 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "C_Z23";
	}
	else if(location.pose.position.x  > 1 && location.pose.position.x <= 2 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "C_Z24";
	}
	else if(location.pose.position.x  > 1 && location.pose.position.x <= 2 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "C_Z25";
	}
	else if(location.pose.position.x  > 0 && location.pose.position.x <= 1 && location.pose.position.y >= 0 && location.pose.position.y < 1){
		zone_name = "C_Z11";
	}
	else if(location.pose.position.x  > 0 && location.pose.position.x <= 1 && location.pose.position.y >= 1 && location.pose.position.y < 2){
		zone_name = "C_Z12";
	}
	else if(location.pose.position.x  > 0 && location.pose.position.x <= 1 && location.pose.position.y >= 2 && location.pose.position.y < 3){
		zone_name = "C_Z13";
	}
	else if(location.pose.position.x  > 0 && location.pose.position.x <= 1 && location.pose.position.y >= 3 && location.pose.position.y < 4){
		zone_name = "C_Z14";
	}
	else if(location.pose.position.x  > 0 && location.pose.position.x <= 1 && location.pose.position.y >= 4 && location.pose.position.y < 5){
		zone_name = "C_Z15";
	}
	else{
		zone_name = "invalid";
	}
}

bool look_for_tag(ros::NodeHandle n, ros::ServiceClient client, img_proc::Find_tag_Srv srv, ros::Publisher pub_mps_name, ros::Publisher pub_mps_zone, ros::Publisher pub_mps_ori)
{
	if(!m_team_set){
		return false;
	}
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::PoseStamped station_pos;
    station_pos.header.frame_id = "/map";
    station_pos.pose.position.x = 0.0;
	station_pos.pose.position.y = 0.0;
	station_pos.pose.position.z = 0.0;
	station_pos.pose.orientation.x = 0.0;
	station_pos.pose.orientation.y = 0.0;
	station_pos.pose.orientation.z = 0.0;
	station_pos.pose.orientation.w = 0.0;

	geometry_msgs::PoseStamped robotsito_pos;
    robotsito_pos.header.frame_id = "/map";
    robotsito_pos.pose.position.x = 0.0;
	robotsito_pos.pose.position.y = 0.0;
	robotsito_pos.pose.position.z = 0.0;
	robotsito_pos.pose.orientation.x = 0.0;
	robotsito_pos.pose.orientation.y = 0.0;
	robotsito_pos.pose.orientation.z = 0.0;
	robotsito_pos.pose.orientation.w = 0.0;
	tag_flag = false;
	std::cout << "\n Look for Tag" << std::endl;
	srv.request.is_find_tag_enabled = true;
	if(client.call(srv)){
		tag_flag = srv.response.success;
		std::cout << "Request accepted" << std::endl;
		if(tag_flag == true){
			mps_name = srv.response.mps_name;
			mps_PointStamped = srv.response.point_stamped;
			//print_vector(mps_PointStamped);
			std::cout << "Saving MPS info" << std::endl;
			if(mps_name.size())
			{
				try{
					listener.waitForTransform("/Log_origin", mps_name[0], ros::Time(0), ros::Duration(1.0));
					listener.lookupTransform("/Log_origin", mps_name[0], ros::Time(0), transform);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
				station_pos.pose.position.x = transform.getOrigin().x();

				try{
					listener.waitForTransform("/Log_origin", "base_link", ros::Time(0), ros::Duration(1.0));
					listener.lookupTransform("/Log_origin", "base_link", ros::Time(0), transform);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
				station_pos.pose.position.x = transform.getOrigin().x();

				if(m_is_cyan){
					station_pos.pose.position.x -= 2;
				} else {
					station_pos.pose.position.x += 2;
				}
				station_pos.pose.position.y = transform.getOrigin().y();

				std::cout << "Station \t" << mps_name[0] << std::endl;	
				define_zone(station_pos);
				std::cout << "\n MPS_Zone " << zone_name << std::endl;
				std::cout << "\n Coords " << station_pos << " w " << station_pos << " z " << std::endl;
				std_msgs::String orientation;
				float norte,oeste,este,noreste,noroeste,offset,sureste,suroeste;
				norte = 0.70;
				oeste = 0.0;
				este = 1;
				noreste = 0.015;
				noroeste = 0.345;
				suroeste = -0.4;
				sureste = 0.9;
				offset = 0.05;
				if(station_pos.pose.orientation.w <= norte + offset  && station_pos.pose.orientation.w >= norte - offset){						
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Norte " << zone_name << std::endl;
						orientation.data = "90"; // si veo Output
					}
					else{
						std::cout << "\n Norte " << zone_name << std::endl;
						orientation.data = "270"; // si veo Input
					}

					std::cout << station_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= este + offset  && station_pos.pose.orientation.w >= este - offset){
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Este " << zone_name << std::endl;
						orientation.data = "0"; // si veo out
					} 
					else{
						std::cout << "\n Este " << zone_name << std::endl;
						orientation.data = "180"; // si veo in
					}
					std::cout << robot_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= oeste + offset  && station_pos.pose.orientation.w >= oeste - offset){
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Oeste " << zone_name << std::endl;
						orientation.data = "180";
					}
					else{
						std::cout << "\n Oeste " << zone_name << std::endl;
						orientation.data = "0"; 
					}
					std::cout << robot_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= noreste + offset  && station_pos.pose.orientation.w >= noreste - offset){
					std::cout << "\n Noreste " << zone_name << std::endl;
					if(contains_str(mps_name[0],"O")){
						orientation.data = "45";
					} 
					else{
						std::cout << "\n Noreste " << zone_name << std::endl;
						orientation.data = "225";
					}
					std::cout << robot_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= noroeste + offset  && station_pos.pose.orientation.w >= noroeste - offset){
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Noroeste " << zone_name << std::endl;
						orientation.data = "135";
					} 
					else{
						std::cout << "\n Noroeste " << zone_name << std::endl;
						orientation.data = "315";
					}
					std::cout << robot_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= sureste + offset  && station_pos.pose.orientation.w >= sureste - offset){
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Sureste " << zone_name << std::endl;
						orientation.data = "315";
					}
					else{
						std::cout << "\n Sureste " << zone_name << std::endl;
						orientation.data = "135";
					}
					std::cout << robot_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
				else if(station_pos.pose.orientation.w <= suroeste + offset  && station_pos.pose.orientation.w >= suroeste - offset){
					if(contains_str(mps_name[0],"O")){
						std::cout << "\n Suroeste " << zone_name << std::endl;
						orientation.data = "225";
					} 
					else{
						std::cout << "\n Suroeste " << zone_name << std::endl;
						orientation.data = "45";
					}
					std::cout << station_pos.pose.orientation.w << " w " << orientation.data << std::endl;
				}
			}
			else
			{
				std::cout << "\n Not identified " << std::endl;
			}
		}
    }
    return tag_flag;
}

bool fwd_n_turn(ros::Publisher pub_cmd_vel, float t_fwd, float t_turn)
{
	std::cout << "\n Movement Start " << std::endl;
	ros::Rate r(10);
	ros::Time end;
	tw_tomap.linear.x=1.0;
	tw_tomap.linear.y=0.0;
	tw_tomap.linear.z=0.0;
	tw_tomap.angular.x=0.0;
	tw_tomap.angular.y=0.0;
	tw_tomap.angular.z=0.0;
	end = ros::Time::now() + ros::Duration(t_fwd);
	while(ros::Time::now() < end){
		//std::cout << "\n Forward " << std::endl;
		pub_cmd_vel.publish(tw_tomap);
	}
	ros::Duration(3);
	tw_tomap.linear.x=0.0;
	tw_tomap.angular.z=1.0;
	end = ros::Time::now() + ros::Duration(t_turn);
	while(ros::Time::now() < end){
		//std::cout << "\n Turn Left" << std::endl;
		pub_cmd_vel.publish(tw_tomap);
	}
	ros::Duration(5);
	std::cout << "\n Movement Finished " << std::endl;
	return true;
}

int main(int argc, char** argv){
	ros::Time::init();
	int curr_pip = 0;
	int pips_vis = 0;
	int turn_step_pip = 0;
	int turn_step_pii = 0;
	int n_steps_pip = 1;
	float step_size = 0.715;
	
	std::cout << "INITIALIZING EXPLORATION NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);

    //Subscribers and Publishers
    ros::Subscriber sub_move_goal_status   	= n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber subLaserScan 			= n.subscribe("/scan", 1, callbackLaserScan);
    ros::Subscriber sub_time_over 			= n.subscribe("/time_over", 1, callback_time_over);

	ros::Publisher pub_mps_name 	= n.advertise<std_msgs::String>("/mps_name", 1000, true);
	ros::Publisher pub_mps_zone		= n.advertise<std_msgs::String>("/mps_zone", 1000,true);
	ros::Publisher pub_mps_ori		= n.advertise<std_msgs::String>("/mps_ori", 1000,true);

	ros::Publisher pub_cmd_vel      = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher pub_zone_goal 	= n.advertise<std_msgs::String>("/zone_goal", 1000, true);
	ros::Publisher pub_mps_pos 	= n.advertise<geometry_msgs::PoseStamped>("/goal", 1000, true);
	
    ros::ServiceClient client = n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");
    img_proc::Find_tag_Srv srv;

    ros::Rate loop(10);

    // PIPs Magenta array
	for(int i=0; i<x_pips_m.size(); i++){
    	tf_pips_m.header.frame_id = "/map";
	    tf_pips_m.pose.position.x = x_pips_m.at(i);
	    tf_pips_m.pose.position.y = y_pips_m.at(i);
		tf_pips_m.pose.position.z = 0;
		tf_pips_m.pose.orientation.x = 0;
		tf_pips_m.pose.orientation.y = 0;
		tf_pips_m.pose.orientation.z = 0;
		tf_pips_m.pose.orientation.w = 0;
		pips_poses.push_back(tf_pips_m);
	}

	// PIPs Cyan array
	for(int i=0; i<x_pips_c.size(); i++){
    	tf_pips_c.header.frame_id = "/map";
	    tf_pips_c.pose.position.x = x_pips_c.at(i);
	    tf_pips_c.pose.position.y = y_pips_c.at(i);
		tf_pips_c.pose.position.z = 0;
		tf_pips_c.pose.orientation.x = 0;
		tf_pips_c.pose.orientation.y = 0;
		tf_pips_c.pose.orientation.z = 0;
		tf_pips_c.pose.orientation.w = 0;
		pips_poses.push_back(tf_pips_c);
	}

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

	location_map.pose.position.x = 0.0;
	location_map.pose.position.y = 0.0;
	location_map.pose.position.z = 0.0;
	location_map.pose.orientation.x = 0.0;
	location_map.pose.orientation.y = 0.0;
	location_map.pose.orientation.z = 0.0;
	location_map.pose.orientation.w = 0.0;

	Log_origin.pose.position.x = 0.0;
	Log_origin.pose.position.y = 0.0;
	Log_origin.pose.position.z = 0.0;
	Log_origin.pose.orientation.x = 0.0;
	Log_origin.pose.orientation.y = 0.0;
	Log_origin.pose.orientation.z = 0.0;
	Log_origin.pose.orientation.w = 0.0;

	robot_curr_pos.pose.position.x = 0.0;
	robot_curr_pos.pose.position.y = 0.0;
	robot_curr_pos.pose.position.z = 0.0;
	robot_curr_pos.pose.orientation.x = 0.0;
	robot_curr_pos.pose.orientation.y = 0.0;
	robot_curr_pos.pose.orientation.z = 0.0;
	robot_curr_pos.pose.orientation.w = 0.0;

	robot_next_pos.pose.position.x = 0.0;
	robot_next_pos.pose.position.y = 0.0;
	robot_next_pos.pose.position.z = 0.0;
	robot_next_pos.pose.orientation.x = 0.0;
	robot_next_pos.pose.orientation.y = 0.0;
	robot_next_pos.pose.orientation.z = 0.0;
	robot_next_pos.pose.orientation.w = 0.0;

	robot2mps.pose.position.x = 0.0;
	robot2mps.pose.position.y = 0.0;
	robot2mps.pose.position.z = 0.0;
	robot2mps.pose.orientation.x = 0.0;
	robot2mps.pose.orientation.y = 0.0;
	robot2mps.pose.orientation.z = 0.0;
	robot2mps.pose.orientation.w = 0.0;

	robot_last_pos.pose.position.x = 0.0;
	robot_last_pos.pose.position.y = 0.0;
	robot_last_pos.pose.position.z = 0.0;
	robot_last_pos.pose.orientation.x = 0.0;
	robot_last_pos.pose.orientation.y = 0.0;
	robot_last_pos.pose.orientation.z = 0.0;
	robot_last_pos.pose.orientation.w = 0.0;

	robot_pos.pose.position.x = 5.5;
	robot_pos.pose.position.y = 1.5;
	robot_pos.pose.position.z = 0.0;
	robot_pos.pose.orientation.x = 0.0;
	robot_pos.pose.orientation.y = 0.0;
	robot_pos.pose.orientation.z = 0.0;
	robot_pos.pose.orientation.w = 0.0;

    goal_tosend.header.frame_id = "/map";
    goal_tosend.pose.position.x = 0.0;
	goal_tosend.pose.position.y = 0.0;
	goal_tosend.pose.position.z = 0.0;
	goal_tosend.pose.orientation.x = 0.0;
	goal_tosend.pose.orientation.y = 0.0;
	goal_tosend.pose.orientation.z = 0.0;
	goal_tosend.pose.orientation.w = 0.0;

	while(ros::ok() && !fail && !success && !time_over.data){
	    switch(state){
			case SM_INIT:{
	    		std::cout << "\n Exploration STAGE: SM_INIT" << std::endl;
				field_color_coords("MAGENTA");
	            std::cout << "I am ready - Exploration route has  " << x_pips.size() << " points" << std::endl;
	    		//state = SM_NAV_PIPS;
				state = SM_NAV_PIPS;
	    		break;
			}

			case SM_NAV_PIPS:{
				std::cout << "\n State machine: SM_NAV_PIPS" << std::endl;
				if(curr_pip <= x_pips.size()){
					std::cout << "Navigating PIP \t" << curr_pip << "\t" << pips_as_zones[curr_pip].data << "\n" << std::endl;
					nav_zone_to_cords(n, pips_as_zones[curr_pip].data, goal_tosend);
					//navigate_to_location(goal_tosend);
					state = SM_TURN_AROUND;
				}
				else{
					std::cout << "All PIPS Visited\t" << std::endl;
					state = SM_FINAL_STATE;
				}
				break;
			}

			case SM_TURN_AROUND:{
				std::cout << "\n State machine: SM_TURN_AROUND" << std::endl;
	 			std::cout << "Turning for PIP \t" << curr_pip << std::endl;
				for(turn_step_pip = 0; turn_step_pip <= n_steps_pip; turn_step_pip++){
					if(fwd_n_turn(pub_cmd_vel, 0.0,step_size))
					{
						std::cout << "Movement Done" << std::endl;
					}
					std::cout << "Step \t" << turn_step_pip << std::endl;
					ros::Duration(3, 0).sleep();
					tag_flag = look_for_tag(n, client, srv, pub_mps_name, pub_mps_zone, pub_mps_ori);
					if(tag_flag){
						std::cout << "Yes Tag" << std::endl;
						state = SM_TAG_DETECTED;
					}
					else{
						std::cout << "No Tag" << std::endl;
					}
				}
				if(turn_step_pip > n_steps_pip){
					curr_pip++;
					std::cout << "Steps finished" << std::endl;
					state = SM_NAV_PIPS;
					}
				break;
			}

			case SM_TAG_DETECTED:{
				std::cout << "\n State machine: SM_TAG_DETECTED" << std::endl;
	    		//Scan Tag and Send Information
	            print_vector(mps_name);
				print_vector(mps_PointStamped);
				try{
					listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
					listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
				robot_pos.pose.position.x = transform.getOrigin().x();
				robot_pos.pose.position.y = transform.getOrigin().y();
				robot_pos.pose.position.z = transform.getOrigin().z();
				robot_pos.pose.orientation.x = transform.getRotation().x();
				robot_pos.pose.orientation.y = transform.getRotation().y();
				robot_pos.pose.orientation.z = transform.getRotation().z();
				robot_pos.pose.orientation.w = transform.getRotation().w();
				
				state = SM_FINAL_STATE;
	    		break;
			}
			
	    	case SM_FINAL_STATE:{
	    		//Finish
	    		std::cout << "\n State machine: SM_FINAL_STATE" << std::endl;	
	            std::cout << "Exploration finished" << std::endl;
	            //std::cout << "\n PIPS successfully visited \t" << pips_vis << "PIPS missed \t" << x_pips.size() - pips_vis << std::endl;
	            //std::cout << "\n Machine information" << std::endl;
	            print_vector(mps_name);
				print_vector(mps_PointStamped);
				try{
					listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
					listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
				robot_last_pos.pose.position.x = transform.getOrigin().x();
				robot_last_pos.pose.position.y = transform.getOrigin().y();
				robot_last_pos.pose.position.z = transform.getOrigin().z();
				std::cout << "Last Pose \n" << robot_last_pos.pose.position << std::endl;
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
