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

#include <festino_arm_moveit_demos/srv_arm.h>
#include <img_proc/MPS_Detector.h>


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
	SM_FIRSTMAPPING,
	SM_VER_FREE_PATH,
	SM_TURN_LEFT,
	SM_TURN_RIGHT,
	SM_NAV_HOME,
	SM_NAV_PIPS,
	SM_TURN_AROUND_PIPS,
	SM_NAV_PIIS,
	SM_TURN_AROUND_PIIS,
	SM_TAG_DETECTED,
	SM_NAV_INPUT,
	SM_GO_ZONE,
	SM_NAV_FWD,
	SM_NAV_AROUND_OBST,
	SM_TAG_SEARCH,
	SM_GIRO,
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
std_msgs::String mps_name_anterior;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool mps_flag;
std::vector<std_msgs::String> mps_names;
bool flag_names = false;
std_msgs::String mps_id;
geometry_msgs::PoseStamped robot_base_pos;


bool nav_flag = false;

//look for tag function
bool tag_flag = false;
std::vector<std::string> mps_name;
std::vector<geometry_msgs::PointStamped> mps_PointStamped;
//look for tag function

//Logistics zones
// Anterior
std::vector<geometry_msgs::PoseStamped> zones_poses;
geometry_msgs::PoseStamped tf_zone;


// PIPs
std::vector<geometry_msgs::PoseStamped> pips_poses;
geometry_msgs::PoseStamped tf_pips;
std_msgs::String pips_as_zones[4];

// PIIs
// Los piis no pueden ser as zones, son cords
std::vector<geometry_msgs::PoseStamped> piis_poses;
geometry_msgs::PoseStamped tf_piis;

sensor_msgs::LaserScan laserScan;
bool flag_free_path = false;

geometry_msgs::Twist tw_tomap;

bool act = false;

void navigate_to_location(ros::NodeHandle n, float x_piis, float y_piis, ros::Publisher pub_rosnav_goal, float timeout)
{
    std::cout << "Coords recieved, publishing coords to navigate \n" << x_piis << "," << y_piis << std::endl;
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

	geometry_msgs::PoseStamped goal_tosend;
    goal_tosend.header.frame_id = "/map";
    goal_tosend.pose.position.x = x_piis;
	goal_tosend.pose.position.y = y_piis;
	goal_tosend.pose.position.z = 0.0;
	goal_tosend.pose.orientation.x = 0.0;
	goal_tosend.pose.orientation.y = 0.0;
	goal_tosend.pose.orientation.z = 0.0;
	goal_tosend.pose.orientation.w = 0.0;

	std::cout << goal_tosend.pose << std::endl;
	pub_rosnav_goal.publish(goal_tosend);


	ros::Duration(timeout).sleep();

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
	std::cout << goal_tosend.pose << std::endl;

	if((goal_tosend.pose.position.x - robot_pos.pose.position.x) < 1.0 && (goal_tosend.pose.position.y - robot_pos.pose.position.y) < 1.0)
	{
		std::cout << "Sí llegué" << std::endl;
		std::cout << "Coords goal \n" << goal_tosend.pose.position << "Coords robot \n" << robot_pos.pose.position << std::endl;
		std::cout << "Diff en x: " << goal_tosend.pose.position.x - robot_pos.pose.position.x << std::endl;
		std::cout << "Diff en y: " << goal_tosend.pose.position.y - robot_pos.pose.position.y << std::endl;
	}
	else
	{
		std::cout << "No llegué, lo siento :c" << std::endl;
	}

}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;
	int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
	int range_left = 0, range_right = 0;
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
        flag_free_path = true;
        //std::cout<<"fwd"<<std::endl;
    }
    else
    {
        flag_free_path = false;
        //std::cout<<"not"<<std::endl;
    }
}

							//---------TESIS---------//
//Se toman las coordenadas de acuerdo con los  propuestos
//en Mittesis: 8 PIPrincipales y 11 PIIntermedios:

//Agrego el punto de salida en cada mitad como primer elemento del arreglo


//MAGENTA

//HOME: (-4.5,0.5)

//PIPs:    M_Z72,      M_Z18,      M_Z78,    M_Z12
//PIPs: (-6.5,1.5), (-0.5,7.5), (-6.5,7.5), (-0.5,1.5)

//X respecto al mapa
std::vector<float> x_pips_m {-4.5, -6.5, -0.5, -6.5, -0.5};
//Y respecto al mapa
std::vector<float> y_pips_m {0.5, 1.5, 7.5, 7.5, 1.5}; 

//PIIs: (-5,3), (-2,6), (-3.5,7.5), (-5,6), (-2,3)

//X respecto al mapa
std::vector<float> x_piis_m {0, -5, -2, -3.5, -5, -2, 0};
//Y respecto al mapa
std::vector<float> y_piis_m {0, 3, 6, 7.5, 6, 3, 4.5}; 

//CIAN

//HOME: (4.5,0.5)

//PIPs:    C_Z18,     C_Z72,    C_Z78,    C_Z12
//PIPs: (0.5,7.5), (6.5,1.5), (6.5,7.5), (0.5,1.5)

//X respecto al mapa
std::vector<float> x_pips_c {4.5, 0.5, 6.5, 6.5, 0.5};
//Y respecto al mapa
std::vector<float> y_pips_c {0.5, 7.5, 1.5, 7.5, 1.5}; 

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


// Este es para NavigationCH
//Lo comente, pero lo llama despues
//Revisar si es necesario o no 
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

/*void callback_mps_name(const std_msgs::String::ConstPtr& msg){
    mps_name2send = *msg;
}*/

void callback_mps_data(const std_msgs::String::ConstPtr& msg){
    mps_data2send = *msg;
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

/*void navigate_to_location(geometry_msgs::PoseStamped location)
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
}*/

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

bool look_for_tag(ros::NodeHandle n, ros::ServiceClient client, img_proc::Find_tag_Srv srv)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tag_flag = false;
	std::cout << "\n Look for Tag" << std::endl;
	srv.request.is_find_tag_enabled = true;
	if(client.call(srv)){
		tag_flag = srv.response.success;
		std::cout << "Request accepted" << std::endl;
		if (tag_flag == true){
			std::cout << "Tag found" << std::endl;
			mps_name = srv.response.mps_name;
			mps_PointStamped = srv.response.point_stamped;
		}
		try{
			listener.waitForTransform("/camera_link",mps_name[0], ros::Time(0), ros::Duration(1000.0));
			listener.lookupTransform("/camera_link", mps_name[0],ros::Time(0), transform);
		}	
		catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		std::cout << "Station \t" << mps_name[0] << std::endl;
		det_mps.pose.position.x = transform.getOrigin().x();
		det_mps.pose.position.y = transform.getOrigin().y();
		det_mps.pose.position.z = transform.getOrigin().z(); 
		std::cout << "\n MPS_Position \n" << det_mps.pose.position << std::endl; //<< det_mps << std::endl;
		define_zone(det_mps);
		std::cout << "\n MPS_Zone " << zone_name << std::endl;
    }
    return tag_flag;
}

void publish_info(ros::Publisher pub_mps_pos, ros::Publisher pub_mps_name){
	std::cout <<"Aquí voy a publicar :D"<<std::endl;
}

bool fwd_n_turn(ros::Publisher pub_cmd_vel, float t_fwd, float t_turn)
{
	std::cout << "\n FWD for " << t_fwd << "secs" << std::endl;
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
	std::cout << "\n TURN for " << t_turn << "secs" << std::endl;
	tw_tomap.linear.x=0.0;
	tw_tomap.angular.z=1.0;
	end = ros::Time::now() + ros::Duration(t_turn);
	while(ros::Time::now() < end){
		//std::cout << "\n Turn Left" << std::endl;
		pub_cmd_vel.publish(tw_tomap);
	}
	return true;
}
int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	bool giro =false;
	bool from_pip = false;
	int opcion = 0;
	int num_giros = 0;
	//int n_pips = 7;
	//int n_piis = 10;
	int n_pips = 4;
	int n_piis = 6;
	int curr_pip = 0;
	int curr_pii = 0;
	int turn_step_pip = 0;
	int turn_step_pii = 0;
	int n_steps_pip = 2;
	int n_steps_pii = 7;
	int quadrant = 0;
	float direction = 0;
	float step_size = 0;
	float angle = 0;

	pips_as_zones[0].data = "/M_Z72";
	pips_as_zones[1].data = "/M_Z18";
	pips_as_zones[2].data = "/M_Z78";
	pips_as_zones[3].data = "/M_Z12";
	
	std::cout << "INITIALIZING EXPLORATION NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);

    //Subscribers and Publishers
    //ros::Subscriber subRefbox 				= n.subscribe("/zones_refbox", 1, callback_refbox_zones);
    ros::Subscriber sub_move_goal_status   	= n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    //ros::Subscriber sub_mps_flag     		= n.subscribe("/aruco_det", 10, callback_mps_flag);
    ros::Subscriber subLaserScan 			= n.subscribe("/scan", 1, callbackLaserScan);
	//MitChanges (Last slot)
    //ros::Subscriber sub_mps_name     		= n.subscribe("/mps_name", 10, callback_mps_name);
	//ros::Subscriber sub_mps_data     		= n.subscribe("/mps_data", 10, callback_mps_data);
	//MitChanges (Last slot)

    
    ros::Publisher pub_goal 		= n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);
	ros::Publisher pub_mps_name 	= n.advertise<std_msgs::String>("/mps_name", 1000, true);
	ros::Publisher pub_mps_pos 		= n.advertise<geometry_msgs::PoseStamped>("/mps_pos", 1000); //, latch=True);
	ros::Publisher pub_cmd_vel      = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher pub_zone_goal 	= n.advertise<std_msgs::String>("/zone_goal", 1000, true);
	ros::Publisher pub_rosnav_goal 	= n.advertise<geometry_msgs::PoseStamped>("/goal", 1000, true);
	
    ros::ServiceClient client = n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");
    img_proc::Find_tag_Srv srv;

    ros::ServiceClient client2 = n.serviceClient<festino_arm_moveit_demos::srv_arm>("/srv_arm");
    festino_arm_moveit_demos::srv_arm srv2;

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
	bool from_left = false;
	bool from_right = false;

    //Robados
    /*std::vector<std::string> mps_name;
    std::vector<geometry_msgs::PointStamped> mps_PointStamped;*/
    //Robados

	// TF initialization
	// Se tiene que a fuerza inicializar las poseStamped porque marca error si no se hace
    
    // Anterior
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

	// Lo dejaré por el momento solo con las coordenadas magentas
    // PIPs array
	for(int i=0; i<x_pips_m.size(); i++){
    	tf_pips.header.frame_id = "/map";
	    tf_pips.pose.position.x = x_pips_m.at(i);
	    tf_pips.pose.position.y = y_pips_m.at(i);
		tf_pips.pose.position.z = 0;
		tf_pips.pose.orientation.x = 0;
		tf_pips.pose.orientation.y = 0;
		tf_pips.pose.orientation.z = 0;
		tf_pips.pose.orientation.w = 0;
		pips_poses.push_back(tf_pips);
	}

	// PIIs array
	for(int i=0; i<x_piis_m.size(); i++){
    	tf_piis.header.frame_id = "/map";
	    tf_piis.pose.position.x = x_piis_m.at(i);
	    tf_piis.pose.position.y = y_piis_m.at(i);
		tf_piis.pose.position.z = 0;
		tf_piis.pose.orientation.x = 0;
		tf_piis.pose.orientation.y = 0;
		tf_piis.pose.orientation.z = 0;
		tf_piis.pose.orientation.w = 0;
		piis_poses.push_back(tf_piis);
	}

	int rotacion = 0;

	//Robados
	tf::TransformListener listener;
	tf::StampedTransform transform;
	//Robados

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

	robot_base_pos.pose.position.x = 0.0;
	robot_base_pos.pose.position.y = 0.0;
	robot_base_pos.pose.position.z = 0.0;
	robot_base_pos.pose.orientation.x = 0.0;
	robot_base_pos.pose.orientation.y = 0.0;
	robot_base_pos.pose.orientation.z = 0.0;
	robot_base_pos.pose.orientation.w = 0.0;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:{
	    		std::cout << "\n Exploration STAGE: SM_INIT" << std::endl;	
	            voice = "I am ready for the exploration stage";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
				ros::Duration(2, 0).sleep();
	    		state = SM_VER_FREE_PATH;
	    		break;
			}

			case SM_FIRSTMAPPING:{
				std::cout << "\n State machine: SM_FIRSTMAPPING" << std::endl;
				if(fwd_n_turn(pub_cmd_vel, 1.0,2.5))
				{
					std::cout << "Movement Done" << std::endl;
				}
				state = SM_VER_FREE_PATH;
				break;
			}

			case SM_VER_FREE_PATH:{
				std::cout << "\n State machine: SM_VER_FREE_PATH" << std::endl;
				if(flag_free_path)
				{
					std::cout << "FWD" << std::endl;
					state = SM_NAV_FWD;	
				}
				else
				{
					std::cout << "RIGHT" << std::endl;
					state = SM_TURN_RIGHT;
				}
				break;
			}

			case SM_NAV_FWD:{
				std::cout << "\n State machine: SM_NAV_FWD" << std::endl;
				std::cout << "Avanza" << std::endl;
				fwd_n_turn(pub_cmd_vel, 2.0, 0.0);
				curr_pip++;
				curr_pii++;
				state = SM_VER_FREE_PATH;
				break;
			}

			case SM_TURN_LEFT:{
				std::cout << "\n State machine: SM_TURN" << std::endl;
				from_left = true;
				fwd_n_turn(pub_cmd_vel, 0.0, 2.0);
				state = SM_VER_FREE_PATH;
				break;
			}

			case SM_TURN_RIGHT:{
				std::cout << "\n State machine: SM_TURN" << std::endl;
				from_right = true;
				fwd_n_turn(pub_cmd_vel, 0.0, 4.0);
				state = SM_VER_FREE_PATH;
				break;
			}

			case SM_NAV_PIPS:{
				std::cout << "\n State machine: SM_NAV_PIPS" << std::endl;
				//Navega a las zonas recorriendo los arreglos pips_poses y piis_poses
				//El contador es el índice que recorre el arreglo
				//Si aún no se han recorrido los puntos de inspeccion sigue 
				if(curr_pip <= n_pips){
					std::cout << "Navigating PIP \t" << curr_pip << "\n" << pips_poses.at(curr_pip) << "\n" << std::endl;
					//navigate_to_location(pips_poses.at(curr_pip));
					pub_zone_goal.publish(pips_as_zones[curr_pip]);
					state = SM_NAV_PIIS;
				}
				else{
					std::cout << "All PIPS Visited - Finishing SM \t" << std::endl;
					state = SM_FINAL_STATE;
				}
				break;
			}

			case SM_TURN_AROUND_PIPS:{
				std::cout << "\n State machine: SM_TURN_AROUND_PIPS" << std::endl;
				// MitComment: Da un giro de 90 grados (2pi) para escanear un cuadrante
				// MC: la direccion depende del cuadrante
				// MC: 3 steps ==> 2*pi/8 = 0.7854
				// MC: ¿porqué pusimos 0.715?  creo que vimos que el robot
	 			// MC: no giraba los 45 grados y encontramos que lo hacía 
	 			// MC: con ese num
	 			step_size = 0.715;
	 			from_pip = true;
	 			std::cout << "Turn arooound for PIP \t" << curr_pip << std::endl;
				if(curr_pip == 1){// || curr_pip == 8){ // Para Magenta y Cyan descomentar la segunda condición // Y probar c:c
					quadrant = 1;
					std::cout << "Looking for ARUCO Tag in quadrant  " << quadrant << std::endl;
					// MC: viendo hacia el frente, giro a la der en 2 steps de 45 grad
					direction = -1;
					angle = step_size*direction;
					
					for(turn_step_pip = 0; turn_step_pip <= n_steps_pip; turn_step_pip++){
						FestinoNavigation::moveDistAngle(0.0, angle*turn_step_pip, 1000);
						std::cout << "Step \t" << turn_step_pip << "\t Angle \t" << angle*turn_step_pip << std::endl;
						std::cout << "Looking for ARUCO   " << std::endl;
						ros::Duration(2, 0).sleep();
						tag_flag = look_for_tag(n, client, srv);
						if(tag_flag){
							publish_info(pub_mps_pos, pub_mps_name);
							state = SM_TAG_DETECTED;
						}
						else{
							std::cout << "No Tag" << std::endl;
						}
					}
					if(turn_step_pip > n_steps_pip){
						curr_pip++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIIS;
					}
				}
				else if(curr_pip == 4){// || curr_pip == 6){
					quadrant = 2;
					std::cout << "Looking for ARUCO Tag in quadrant  " << quadrant << std::endl;
					// MC: viendo hacia el frente, giro a la izq 2 steps de 45 grad
					direction = 1;
					angle = step_size*direction;
					for(turn_step_pip = 0; turn_step_pip <= n_steps_pip; turn_step_pip++){
						FestinoNavigation::moveDistAngle(0.0, angle*turn_step_pip, 1000);
						std::cout << "Step \t" << turn_step_pip << "\t Angle \t" << angle*turn_step_pip << std::endl;
						std::cout << "Looking for ARUCO   " << std::endl;
						ros::Duration(2, 0).sleep();
						tag_flag = look_for_tag(n, client, srv);
						if(tag_flag){
							state = SM_TAG_DETECTED;
						}
						else{
							std::cout << "No Tag" << std::endl;
						}
					}
					if(turn_step_pip > n_steps_pip){
						curr_pip++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIIS;
					}
				}
				else if(curr_pip == 2){// || curr_pip == 7){
					quadrant = 3;
					std::cout << "Looking for ARUCO Tag in quadrant  " << quadrant << std::endl;
					// MC: viendo hacia el frente, giro a la izq 90 grad sin steps
					// MC: luego otros 45 a la izq, con 2 steps
					FestinoNavigation::moveDistAngle(0.0, 2*turn_step_pip, 1000);
					direction = 1;
					angle = step_size*direction;
					for(turn_step_pip = 0; turn_step_pip <= n_steps_pip; turn_step_pip++){
						FestinoNavigation::moveDistAngle(0.0, angle*turn_step_pip, 1000);
						std::cout << "Step \t" << turn_step_pip << "\t Angle \t" << angle*turn_step_pip << std::endl;
						std::cout << "Looking for ARUCO   " << std::endl;
						ros::Duration(2, 0).sleep();
						tag_flag = look_for_tag(n, client, srv);
						if(tag_flag){
							state = SM_TAG_DETECTED;
						}
						else{
							std::cout << "No Tag" << std::endl;
						}
					}
					if(turn_step_pip > n_steps_pip){
						curr_pip++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIIS;
					}
				}
				else if(curr_pip == 3){// || curr_pip == 5){
					quadrant = 4;
					std::cout << "Looking for ARUCO Tag in quadrant  " << quadrant << std::endl;
					// MC: viendo hacia el frente, giro a la der 90 grad sin steps
					// MC: luego otros 45 a la der, con 2 steps
					FestinoNavigation::moveDistAngle(0.0, -2*turn_step_pip, 1000);
					direction = -1;
					angle = step_size*direction;
					for(turn_step_pip = 0; turn_step_pip <= n_steps_pip; turn_step_pip++){
						FestinoNavigation::moveDistAngle(0.0, angle*turn_step_pip, 1000);
						std::cout << "Step \t" << turn_step_pip << "\t Angle \t" << angle*turn_step_pip << std::endl;
						std::cout << "Looking for ARUCO   " << std::endl;
						ros::Duration(2, 0).sleep();
						tag_flag = look_for_tag(n, client, srv);
						if(tag_flag){
							state = SM_TAG_DETECTED;
						}
						else{
							std::cout << "No Tag" << std::endl;
						}
					}
					if(turn_step_pip > n_steps_pip){
						curr_pip++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIIS;
					}
				}
				break;
			}

			case SM_NAV_PIIS:{
				std::cout << "\n State machine: SM_NAV_PIIS" << std::endl;
				if(curr_pii <= n_piis){
					std::cout << "Navigating PII \t" << curr_pii << "\n" << piis_poses.at(curr_pii) << "\n" << std::endl;
					//navigate_to_location(pips_poses.at(curr_pii));
					std::cout << "Coords del pii" << x_piis_m[curr_pii] <<","<< y_piis_m[curr_pii] << std::endl;
					std::cout << "Navigating PII \t" << curr_pii << "\n" << piis_poses.at(curr_pii) << "\n" << std::endl;
					navigate_to_location(n,x_piis_m[curr_pii], y_piis_m[curr_pii],pub_rosnav_goal, 10.0);
					state = SM_FINAL_STATE;
				}
				else{
					std::cout << "All PIIS Visited \t" << std::endl;
					state = SM_NAV_PIPS;
				}
				break;
			}

			case SM_TURN_AROUND_PIIS:{
				std::cout << "\n State machine: SM_TURN_AROUND_PIIS" << std::endl;
				// MitComment: Da un giro de 360 grados (2pi) para escanear todo
				// MC: Steps ==> 2*pi/8 = 0.7854
				// MC: ¿porqué pusimos 0.715?  creo que vimos que el robot
	 			// MC: no giraba los 45 grados y encontramos que lo hacía 
	 			// MC: con ese num
	 			step_size = 0.715;
	 			from_pip = false;
	 			std::cout << "Turn arooound for PII \t" << curr_pii << std::endl;
				for(turn_step_pii = 0; turn_step_pii <= n_steps_pii; turn_step_pii++){
					FestinoNavigation::moveDistAngle(0.0, step_size, 1000);
					std::cout << "Step \t" << turn_step_pii << "\t Angle \t" << step_size*turn_step_pii << std::endl;
					std::cout << "Looking for ARUCO   " << std::endl;
					ros::Duration(2, 0).sleep();
					tag_flag = look_for_tag(n, client, srv);
					if(tag_flag){
						state = SM_TAG_DETECTED;
						curr_pii++;
						break;
					}
					else{
						std::cout << "No Tag" << std::endl;
					}
				}
				if(curr_pii == 1 || curr_pii == 4){
					std::cout << "Nav to PII" << std::endl;
					if(turn_step_pii > n_steps_pii){
						curr_pii++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIIS;
					}
				}
				else{
					std::cout << "Nav to PIP" << std::endl;
					if(turn_step_pii > n_steps_pii){
						curr_pii++;
						std::cout << "Steps finished" << std::endl;
						state = SM_NAV_PIPS;
					}
				}
				break;
			}

			case SM_TAG_DETECTED:{
	    		//Scan Tag and Send Information
	    		std::cout << "\n State machine: SM_TAG_DETECTED" << std::endl;
	    		std::cout << "¿imprimo, publico? ¿qué hago?" << std::endl;
	            voice =  "Print information";
	            print_vector(mps_name);
				print_vector(mps_PointStamped);	
				//Vuelve al pi de donde venía
				if(from_pip){
					std::cout << "Back to the route" << std::endl;
					from_pip = false;
					state = SM_TURN_AROUND_PIPS;
				}
				else if(from_pip == false){
					std::cout << "Back to the route" << std::endl;
					from_pip = false;
					state = SM_TURN_AROUND_PIIS;
				}
	    		break;
			}
			
	    	case SM_FINAL_STATE:{
	    		//Finish
	    		std::cout << "\n State machine: SM_FINAL_STATE" << std::endl;	
	            std::cout << "Exploration finished" << std::endl;
	            std::cout << "\n PIPS visited \t" << curr_pip << std::endl;
	            std::cout << "\n PIIS visited \t" << curr_pii << std::endl;
	            std::cout << "\n Machine imformation" << std::endl;
	            print_vector(mps_name);
				print_vector(mps_PointStamped);

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