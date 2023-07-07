//1 Alinearse con la salida de la máquina
//2 Alejarse un poco (30 cm)
//3 Levantar servicio "find piece" (da coordenadas (respecto al kinect) y un booleano de si encontró la pieza)
//4 Convertir las coordenadas que da el servicio, resecto al arm_base_link en vez de que sean respecto al kinect (con TF).
//5 Tomar la pieza
//6 Ir a la entrada y dejar la pieza

#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"
#include <algorithm>
#include <math.h>

//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

#include "img_proc/Find_piece_Srv.h"
#include "Festino_arm_moveit_demos/srv_arm.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

//Se puede cambiar, agregar o eliminar los estados
enum SMState
{
	SM_INIT,
	SM_ALLIGN,
	SM_FIND_PIECE,
	SM_GRASPING_PIECE,
	SM_GO_INPUT,
	SM_GRASPING_DELIVER,
	SM_GO_INIT,
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
geometry_msgs::PoseStamped piece;

geometry_msgs::Twist posNew;

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

float x = 0;
float y = 0;
float z = 0;
float pitch = 0;

bool manipBlocker = false;
bool gripperState = false;

int conde = 0;


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
	std::string cmd_vel_name = "/cmd_vel";

    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000); //, latch=True);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);
    
    ros::ServiceClient client = n.serviceClient<Festino_arm_moveit_demos::srv_arm>("srv_arm");
    Festino_arm_moveit_demos::srv_arm srv;

    ros::ServiceClient clientFindPiece = n.serviceClient<img_proc::Find_piece_Srv>("/vision/find_piece/point_stamped");
    img_proc::Find_piece_Srv srvFindPiece;

    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(cmd_vel_name, 1000);

    ros::Rate loop(10);

    std::string voice;

    std::vector<std::string> mps_name;
    geometry_msgs::PointStamped locProduct;

    //tf2_ros::Buffer

    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};

	 //TF related stuff 
	 //Se tiene que a fuerza inicializar las poseStamped porque marca error si no se hace

	//Contador que lleva el número de zonas que se han recorrido
	int cont = 0;

	//Contador que lleva el número de giros
	int cont_giro = 0;

	while(ros::ok() && !fail && !success){
	    switch(state)
	    {
			case SM_INIT:
			{
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the grasping challenge";
	            std::cout << voice << std::endl;

				//FestinoHRI::say(voice,3);
				state = SM_ALLIGN;

				//arr_values.values = {0,0,0,1,1,1};
                //pub_digital.publish(arr_values);
                break;
              }
			case SM_ALLIGN:
			{
				std::cout << "State machine: SM_ALLIGN" << std::endl;

				//-------------------------------------------
				//-----AQUI VA EL SERVICIO DE ALINEACION-----
				//-------------------------------------------

				//Alling_srv

				//Se mueve tantito para atras para que el kinect pueda ver la pieza
				std::cout<<"Moviendo hacia atras 45 cm"<<std::endl;
				//FestinoNavigation::moveDist(0.45, 30000);
				state = SM_FIND_PIECE;
				break;
			}

			case SM_FIND_PIECE:
			{
				std::cout << "State machine: SM_FIND_PIECE" << std::endl;	

				srvFindPiece.request.is_find_piece_enabled = true;
				if(clientFindPiece.call(srvFindPiece))
				{
					locProduct = srvFindPiece.response.point_stamped;
					
					//std::cout<<"x = "<<srvFindPiece.response.point_stamped.point.x<<std::endl;
					//std::cout<<"y = "<<srvFindPiece.response.point_stamped.point.y<<std::endl;
					//std::cout<<"z = "<<srvFindPiece.response.point_stamped.point.z<<std::endl;

					std::cout<<"para el LOC"<<std::endl<<"x= "<<locProduct.point.x<<std::endl;
					std::cout<<"y= "<<locProduct.point.y<<std::endl;
					std::cout<<"z= "<<locProduct.point.z<<std::endl;

					std::cout<<"frame_id= "<<locProduct.header.frame_id<<std::endl;


					//--------------------------------------------------
					//-----AQUI VA LA TRANSFORMACION DE COORDENADAS-----
					//--------------------------------------------------

				}

		    	state = SM_GRASPING_PIECE;
	    		break;
			}

			case SM_GRASPING_PIECE:
			{
				std::cout << "State machine: SM_GRASPING_PIECE" << std::endl;	

				//Coordenates for pre-grasping
				srv.request.manipBlocker = false;
				//srv.request.x = locProduct.point.x;
				//srv.request.y = locProduct.point.y;
				//srv.request.z = locProduct.point.z;

				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.10;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo (pre-grasp)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo, sad sad sad (pre-grasp)"<<std::endl;
				}

				srv.request.manipBlocker = true;
				srv.request.gripperState = true;

				if(client.call(srv))
				{
					std::cout<<"Abriendo pinza (pre-grasp)"<<std::endl;
				}
				else
				{
					std::cout<<"No pude abrir pinzaaaa (pre-grasp), sad sad sad"<<std::endl;
				}

				ros::Duration(2, 0).sleep();

				//Coordenates for grasping
				srv.request.manipBlocker = false;
				
				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.0475;
				srv.request.pitch = 0.0;				

				if(client.call(srv))
				{
					std::cout<<"Alineando pinza (grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo para alinear pinza (grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(2, 0).sleep();

				//Grasping
				srv.request.manipBlocker = true;
				srv.request.gripperState = false;

				if(client.call(srv))
				{
					std::cout<<"Cerrando pinza (grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee cerrar pinza (grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(1, 0).sleep();
				//Coordenates for pos-grasping
				srv.request.manipBlocker = false;
				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.10;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo (post-grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo (post-grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(2, 0).sleep();

				//------------------------------------------------
				//-----RETRAER BRAZO (coordenadas opcionales)-----
				//------------------------------------------------
				srv.request.manipBlocker = false;
				srv.request.x = 0.1;
				srv.request.y = 0.0;
				srv.request.z = 0.15;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo a guardadito"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo a guardadito, sad sad sad"<<std::endl;
				}
				ros::Duration(1, 0).sleep();
				state = SM_GO_INPUT;
				break;
			}

			case SM_GO_INPUT:
			{
				std::cout << "State machine: SM_GO_INPUT" << std::endl;	
				
				//Se mueve hacia atras 25cm
				//FestinoNavigation::moveDist(-0.45, 30000);
				std::cout << "Se mueve hacia atras 25 cm" << std::endl;	
				posNew.linear.x = -0.25;
				for(float x = 0; x<= posNew.linear.x; x+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}
				
				//Se mueve hacia la derecha 50 cm
				std::cout << "Se mueve hacia la derecha 50 cm" << std::endl;	
				posNew.linear.y = 0.50;
				for(float y = 0; y<= posNew.linear.y; y+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}

				//Se mueve hacia adelante 1 m
				std::cout << "Se mueve hacia adelante 1 m" << std::endl;	
				posNew.linear.x = 1.00;
				for(float x = 0; x<= posNew.linear.x; x+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}

				//Da media vuelta
				FestinoNavigation::moveDistAngle(0.0, M_PI,30000);
				std::cout << "Da media vuelta" << std::endl;	
				state = SM_GRASPING_DELIVER;
				ros::Duration(3, 0).sleep();
				break;
			}

			case SM_GRASPING_DELIVER:
			{
				std::cout << "State machine: SM_GRASPING_DELIVER" << std::endl;	

				//Coordenates for pre-grasping
				srv.request.manipBlocker = false;
				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.10;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo (pre-grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo (pre-grasping), sad sad sad"<<std::endl;
				}

				ros::Duration(2, 0).sleep();
				//Coordenates for grasping
				srv.request.manipBlocker = false;
				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.0475;
				srv.request.pitch = 0.0;				

				if(client.call(srv))
				{
					std::cout<<"Alineando el brazo (pre-grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee alinear el brazo (post-grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(2, 0).sleep();
				//Grasping
				srv.request.manipBlocker = true;
				srv.request.gripperState = true;

				if(client.call(srv))
				{
					std::cout<<"Abriendo pinza (grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee arir pinza (grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(1, 0).sleep();
				//Coordenates for pos-grasping
				srv.request.manipBlocker = false;
				srv.request.x = 0.17;
				srv.request.y = -0.095;
				srv.request.z = 0.10;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo sin pieza (post-grasping)"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo (post-grasping), sad sad sad"<<std::endl;
				}
				ros::Duration(1, 0).sleep();
				//------------------------------------------------
				//-----RETRAER BRAZO (coordenadas opcionales)-----
				//------------------------------------------------
				srv.request.manipBlocker = false;
				srv.request.x = 0.1;
				srv.request.y = 0.0;
				srv.request.z = 0.15;
				srv.request.pitch = 0.0;

				if(client.call(srv))
				{
					std::cout<<"Moviendo el brazo a guardadito"<<std::endl;
				}
				else
				{
					std::cout<<"No pudeeeee mover el brazo a guardadito, sad sad sad"<<std::endl;
				}
				state = SM_GO_INIT;
				ros::Duration(1, 0).sleep();
				break;
			}

			case SM_GO_INIT:
			{
				std::cout << "State machine: SM_GO_INIT" << std::endl;	
				
				//Se mueve hacia atras 25cm
				std::cout << "Se mueve hacia atras 25 cm" << std::endl;
				//FestinoNavigation::moveDist(-0.25, 30000);
				posNew.linear.x = -0.25;
				for(float x = 0; x<= posNew.linear.x; x+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}
				
				//Se mueve hacia la derecha 50 cm
				std::cout << "Se mueve hacia la derecha 50 cm" << std::endl;	
				posNew.linear.y = 0.50;
				for(float y = 0; y<= posNew.linear.y; y+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}

				//Se mueve hacia adelante 1 m
				std::cout << "Se mueve hacia adelante 1 m" << std::endl;
				//FestinoNavigation::moveDist(1.0, 30000);
				posNew.linear.x = 1.00;
				for(float x = 0; x<= posNew.linear.x; x+=0.05)
				{
					pub_cmd_vel.publish(posNew);
					ros::Duration(1,0).sleep();
				}

				//Da media vuelta
				std::cout << "Da media vuelta" << std::endl;	
				FestinoNavigation::moveDistAngle(0.0, M_PI,30000);
				state = SM_GRASPING_DELIVER;
				ros::Duration(3, 0).sleep();

				conde++;
				std::cout<<"Conde -> "<<conde<<std::endl;
				if(conde < 3)
				{
					state = SM_ALLIGN;
					std::cout << "state = "<< state<<std::endl;
					std::cout << "ALINEAAAAR"<<std::endl;
				}
				else
				{
					state = SM_FINAL_STATE;
					std::cout << "state = "<< state<<std::endl;
					std::cout << "TERMINEEEEE "<<std::endl;
				}
				ros::Duration(1, 0).sleep();
				break;
			}

	    	case SM_FINAL_STATE:
	    	{
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