//De la arena completa solo se usa una mitad (Magenta o Cyan) y de esa solo se agarra 5x5.
//La arena completa mide 14 x 8, pero en este caso solo se usará 5x5. 


//¿Qué pasa si una máquina está muy cerca de un punto de escaneo? El kinect no verá el Aruco.
//¿Qué pasa si hay una máquina en un punto de escaneo?
//¿Qué pasa si un aruco está muy lejos para que la nube de puntos lo detecte?

//Explore the Field
//Report the position and orientation of the MPS
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


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"


#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

using namespace std;

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

//MitChanges (Last slot)

std_msgs::String mps_data2send;
std_msgs::String mps_name2send;

//MitChanges (Last slot)

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

bool nav_flag = false;

//Logistics zones
std::vector<geometry_msgs::PoseStamped> zones_poses;
geometry_msgs::PoseStamped tf_zone;

sensor_msgs::LaserScan laserScan;
bool flag_door = true;

geometry_msgs::Twist posNew;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;

    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    std::cout<<laserScan.ranges.size()<<std::endl;
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

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

//Zonas de prueba para el escaneo (se eligieron de forma que cubrieran gran parte del lab)
//M_Z53 //M_Z14 //M_Z22 //M_Z21

//Estas coordenadas se obtuvieron del archivo "challengeTracks_Zones.launch"
//Son las x de las zonas de escaneo respecto al origen de logistics
//std::vector<float> tf_x {-5.5, -1.5, 0.5, 3.5};
//Son las y de las zonas de escaneo respecto al origen de logistics
//std::vector<float> tf_y {2.5, 3.5, 4.5, 3.5}; 


//Son las coordenadas respecto al mapa que considera solo las zonas disponibles
//Estas coordenadas se obtuvieron contando los cuadritos en el rviz respecto al origen del mapa
//Las zonas de arriba son las que se encuentran directamente en el archivo launch
//Son las x de las zonas de escaneo respecto al mapa
//std::vector<float> tf_x {0.5, 4.5, 6.5, 9.5};
//Son las y de las zonas de escaneo respecto al mapa
//std::vector<float> tf_y {-4.5, -3.5, -2.5, -3.5}; 

							//---------COMPETENCIA---------//

//Para la competencia real nos toca el lado Magenta para la prueba de exploration challenge
//Estas coordenadas se obtuvieron del archivo "logisticsZones.launch", las cuales ya están respecto al origen del mapa
//Las 4 zonas que serán usadas son: M_Z42, M_Z22, M_Z24 y M_Z44
//Las otras 4 zonas son las esquinas: M_Z55, M_Z52, M_Z11 y M_Z15

//Son las x de las zonas de escaneo respecto al mapa
std::vector<float> tf_x {-3.5, -1.5, -1.5, -3.5, -4.5, -4.5, -0.5, -0.5};
//Son las y de las zonas de escaneo respecto al mapa
std::vector<float> tf_y {1.5, 1.5, 3.5, 3.5, 4.5, 4.5, 1.5, 0.5, 4.5}; 

void callback_refbox_zones(const std_msgs::String::ConstPtr& msg)
{
    new_zone = *msg;
    target_zones.push_back(new_zone);

    if(target_zones.size() == 12){
        flag_zones = true;
    }
}

//Arreglo con los nombres de las estaciones
/*void callback_mps_name(const std_msgs::String::ConstPtr& msg){
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
}*/


//MitChanges (Last slot)
void callback_mps_name(const std_msgs::String::ConstPtr& msg){
    mps_name2send = *msg;
}

void callback_mps_data(const std_msgs::String::ConstPtr& msg){
    mps_data2send = *msg;
}

//MitChanges (Last slot)

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
			nav_flag = false;
         	//std::cout << "Cannot move to " << std::endl;
                //FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
}

//Estas coordenadas son M_Z32, ahi sabemos que no habra maquina
std::vector<float> arreglo_x = {-2.5, 0.0, 0.0, 0.0, 0.0};
std::vector<float> arreglo_y = {1.5, 0.0, 0.0, 0.0, 0.0};

//M_Z32, M_Z34
//std::vector<float> arreglo_x = {-2.5, -2.5};
//std::vector<float> arreglo_y = {1.5, 3.5};

//M_Z32, M_Z34, M_Z44, M_Z24
//std::vector<float> arreglo_x = {-2.5, -2.5, -3.5, -1.5};
//std::vector<float> arreglo_y = {1.5, 3.5, 3.5, 3.5};

//M_Z32, M_Z22, M_Z24, M_Z44, M_Z42
//std::vector<float> arreglo_x = {-2.5, -1.5, -1.5, -3.5, -3.5};
//std::vector<float> arreglo_y = {1.5, 1.5, 3.5, 3.5, 1.5};



//Los angulos de este arreglo estan en radianes, en grados son: 0, 270, 180, 90
//std::vector<float> arreglo_alfa = {0, 4.71239, 3.14159, 1.5708};

//Los angulos de este arreglo estan en radianes, en grados son: 45, 90, 135, 180, 225, 270, 315, 0
//Avanza de 45 en 45 grados
std::vector<float> arreglo_alfa = {0.7853, 1.5708, 2.356, 3.14159, 3.9269, 4.71239, 5.4977, 0};

//Nuevas coordeandas del aruco respecto al mapa
float aruco_x;
float aruco_y;

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
	bool tag_flag;
	bool giro =false;

	std::cout << "INITIALIZING EXPLORATION NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);

	std::string  cmd_vel_name="/cmd_vel";


    //Subscribers and Publishers
    ros::Subscriber subRefbox 				= n.subscribe("/zones_refbox", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   	= n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_mps_flag     		= n.subscribe("/aruco_det", 10, callback_mps_flag);
    ros::Subscriber subLaserScan 			= n.subscribe("/scan", 1, callbackLaserScan);
	//MitChanges (Last slot)
    ros::Subscriber sub_mps_name     		= n.subscribe("/mps_name", 10, callback_mps_name);
	ros::Subscriber sub_mps_data     		= n.subscribe("/mps_data", 10, callback_mps_data);
	//MitChanges (Last slot)

    
    ros::Publisher pub_goal 		= n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);
	ros::Publisher pub_station_info = n.advertise<std_msgs::String>("/station_info", 1000, true);
	ros::Publisher pub_cmd_vel      = n.advertise<geometry_msgs::Twist>(cmd_vel_name, 1000);


    ros::ServiceClient client = n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");
    img_proc::Find_tag_Srv srv;

    ros::Rate loop(10);

    std::string voice;
    
    
    std::stringstream stream;
    std::string name;
    std::string type;
    std::string zone;
    std::string rot;
    std::string mps_info;


    std::string mps_zone;
    std::string mps_type;

    std_msgs::String mps_info2send;

    std::vector<std::string> mps_name;
    std::vector<geometry_msgs::PointStamped> mps_aruco;

	 //TF related stuff 
	 //Se tiene que a fuerza inicializar las poseStamped porque marca error si no se hace
	for(int i=0; i<tf_x.size(); i++){
    	tf_zone.header.frame_id = "/map";
	    tf_zone.pose.position.x = tf_x.at(i);
	    tf_zone.pose.position.y = tf_y.at(i);
		tf_zone.pose.position.z = 0;
		tf_zone.pose.orientation.x = 0;
		tf_zone.pose.orientation.y = 0;
		tf_zone.pose.orientation.z = 0;
		tf_zone.pose.orientation.w = 0;
		zones_poses.push_back(tf_zone);
	}

	int rotacion = 0;


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


	int num_zonas_visitadas;

	int opcion = 3;

	if (opcion == 0){
		num_zonas_visitadas = 1;
		arreglo_x = {-2.5, 0.0, 0.0, 0.0, 0.0};
		arreglo_y = {1.5, 0.0, 0.0, 0.0, 0.0};
	}
	else if (opcion == 1){
		num_zonas_visitadas = 2;
		arreglo_x = {-2.5, -2.5, 0.0, 0.0, 0.0};
		arreglo_y = {1.5, 3.5, 0.0, 0.0, 0.0};
	}
	else if (opcion == 2){
		num_zonas_visitadas = 4;
		arreglo_x = {-2.5, -2.5, -3.5, -1.5, 0.0};
		arreglo_y = {1.5, 3.5, 3.5, 3.5, 0.0};
	}
	else if (opcion == 3){
		num_zonas_visitadas = 5;
		arreglo_x = {-2.5, -1.5, -1.5, -3.5, -3.5};
		arreglo_y = {1.5, 1.5, 3.5, 3.5, 1.5};
	}

	int num_giros = 7;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:{
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the exploration challenge";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		state = SM_GO_ZONE;
	    		break;
			}
			case SM_GO_ZONE:{
				std::cout << "State machine: SM_GO_ZONE" << std::endl;	
				//Se navega a las zonas recorriendo el arreglo zones_poses
				//El contador es el índice que recorre el arreglo
				//Si aún no se han recorrido las 4 zonas entonces sigue 
 				nav_flag = true;
				if(cont < num_zonas_visitadas){
					// TestComment
					//navigate_to_location(zones_poses.at(cont));


					//Avanza un metro
					// posNew.linear.x = 0.85;
					// for(float x = 0; x <= posNew.linear.x; x+=0.05)
					// {
					// 	std::cout<<x<<std::endl;
					// 	pub_cmd_vel.publish(posNew);
					// 	ros::Duration(1, 0).sleep();	
					// }
					//step++;

					//Avanza un metro
					if(cont == 0){
						FestinoNavigation::moveDist(1, 10000);
						std::cout << "avanza" << std::endl;
					}

					if (opcion == 1){
						if(cont == 1){
							FestinoNavigation::moveDist(2, 10000);	
						}
					}
					else if (opcion == 2){
						
						if(cont == 1){
						   FestinoNavigation::moveDist(2, 10000);
						}
						else if(cont == 2){
						   FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						   ros::Duration(1, 0).sleep();
						   FestinoNavigation::moveDist(1, 10000);
						   ros::Duration(1, 0).sleep();
						   FestinoNavigation::moveDistAngle(0.0, -1.5, 10000);
						}
						else if (cont == 2){
							FestinoNavigation::moveDistAngle(0.0, -1.5, 10000);
							ros::Duration(1, 0).sleep();
						   	FestinoNavigation::moveDist(2, 10000);
							ros::Duration(1, 0).sleep();
							FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						}
					}
					else if (opcion == 3){
						if(cont == 1){
						   FestinoNavigation::moveDistAngle(0.0, -1.5, 10000);
						   ros::Duration(1, 0).sleep();
						   FestinoNavigation::moveDist(1, 10000);
						   FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						}
						else if(cont == 2){
						   FestinoNavigation::moveDist(2, 10000);
						}
						else if (cont == 2){
						   FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						   ros::Duration(1, 0).sleep();
						   FestinoNavigation::moveDist(2, 10000);
						   ros::Duration(1, 0).sleep();
						   FestinoNavigation::moveDistAngle(0.0, -1.5, 10000);
						}
						else if(cont == 3){
							FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						   	ros::Duration(1, 0).sleep();
							FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
							ros::Duration(1, 0).sleep();
							FestinoNavigation::moveDist(2, 10000);
							ros::Duration(1, 0).sleep();
							FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						   	ros::Duration(1, 0).sleep();
							FestinoNavigation::moveDistAngle(0.0, 1.5, 10000);
						}
					}

					//FestinoNavigation::moveDistAngle(0.0, -1.2, 10000);
					// //posNew.linear.x = 0.50;
					// for(float x = 0; x <= posNew.linear.x; x+=0.05)
					// {
					// 	std::cout<<x<<std::endl;
					// 	pub_cmd_vel.publish(posNew);
					// 	ros::Duration(1, 0).sleep();	
					// }
					// FestinoNavigation::moveDistAngle(0.0, -1.2, 10000);
					// //posNew.linear.x = 0.50;
					// for(float x = 0; x <= posNew.linear.x; x+=0.05)
					// {
					// 	std::cout<<x<<std::endl;
					// 	pub_cmd_vel.publish(posNew);
					// 	ros::Duration(1, 0).sleep();	
					// }
					// //posNew.linear.x = 0.50;
					// FestinoNavigation::moveDistAngle(0.0, -1.2, 10000);
					// for(float x = 0; x <= posNew.linear.x; x+=0.05)
					// {
					// 	std::cout<<x<<std::endl;
					// 	pub_cmd_vel.publish(posNew);
					// 	ros::Duration(1, 0).sleep();	
					// }
					ros::Duration(1, 0).sleep();
		            //TestComment
					// if(nav_flag){
					// 	std::cout << "Im in Zone  " << cont << std::endl;
					// 	std::cout << "Im in coordinates " << zones_poses.at(cont) << std::endl;
					// 	state = SM_SCAN_SPACE;
					// }
					// else{
					// 	std::cout << "I can't go to zone " << cont << std::endl;
					// 	cont++;
					// 	std::cout << "I'll nav to zone " << cont << std::endl;
					// 	state = SM_GO_ZONE;
					// }	
					state = SM_SCAN_SPACE;
				}
				//Cuando se hayan recorrido las 4 zonas ya terminó
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

				if(mps_name2send.data != ""){
					try{
						listener.waitForTransform(mps_name2send.data, "/base_link", ros::Time(0), ros::Duration(1000.0));
						listener.lookupTransform(mps_name2send.data, "/base_link", ros::Time(0), transform);
					}
					catch(tf::TransformException ex){
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
					}
				}
				det_mps.pose.position.x = -transform.getOrigin().x();
				det_mps.pose.position.y = -transform.getOrigin().y();
				det_mps.pose.position.z = -transform.getOrigin().z();


				std::cout << det_mps.pose.position << std::endl; //<< det_mps << std::endl;

				transform_aruco_map(det_mps);

				define_zone(location_map);
				std::cout << zone_name << std::endl;

				mps_info2send.data = mps_data2send.data +","+ zone_name + ",-1";

				if(zone_name != "invalid")
				{
					pub_station_info.publish(mps_info2send);
				}
				else
				{
					std::cout << "zone_name = invalid" << std::endl;

				}

				ros::Duration(2.0).sleep();

				/*Habilita el servicio y prende el kinect
	            srv.request.is_find_tag_enabled = true;
				tag_flag = false;
				if (client.call(srv))
				{
					tag_flag = srv.response.success;
					if (tag_flag == true)
					{
						mps_name = srv.response.mps_name;
						mps_aruco = srv.response.point_stamped;
						//mps_zone = "M_Z54";
						//mps_type = "CS";
					

						if(no alcanza nube de puntos){
							FestinoNavigation::moveDist(3, 30000);
						}

						for(int i = 0; i < mps_name.size(); i++)
						{
							stream << rotacion;
							stream >> rot;

							mps_info = mps_name[i] + "," + mps_type + "," + mps_zone + "," + rot;
							mps_id.data = mps_info;
							std::cout << "string info\n" << mps_info << std::endl;
							//std::cout << "Elem - pos\n" << mps_aruco[i].point << std::endl;	
							station_info_pub.publish(mps_id);
							ros::Duration(2, 0).sleep();
							//station_zone_pub.publish(voice);
						}
						print_vector(mps_names,",");
						state = SM_TAG_DETECTED;
					}
					else{
						state = SM_GIRO;
					}
					
					//mps_name = "Not Identified";
					/*if(tag_flag and mps_name != srv.responSM_GIROse.mps_name)

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
					}
				}
				else
				{
					ROS_ERROR("Failed to call service to search Tag");
					return 1;
				}*/
	            /*voice.data = msg;
	            pub_speaker.publish(voice);
	            ros::Duration(3, 0).sleep();*/

				// ----- If Tag Detected -> Send Information and discard as go_to_obstacle -----
				// ----- Else -> Continue to navigate -----
				state = SM_GIRO;
	            break;
	        }
			case SM_GIRO:{
				std::cout << "State machine: SM_GIRO" << std::endl;
				std::cout << "Contador giro" << cont_giro << std::endl;
				if(cont_giro < num_giros){
					std::cout << "Turn arooound" << std::endl;
					//Da un giro de 360 grados (2pi) para escanear todo
					//Le puse un ángulo de 6.2832 pero no funciona así

					//TestComment
					FestinoNavigation::moveDistAngle(0.0, -0.715, 10000);
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
					cont_giro = 0;
					//Ultimo giro para quedar mirando hacia en frente 
					FestinoNavigation::moveDistAngle(0.0, -0.715, 10000);
					state = SM_GO_ZONE;
				}
				break;
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