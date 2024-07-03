//State Machine for main track
#include<iostream>
#include <cmath>
#include <math.h>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"
#include <algorithm>

//Para encontrar piezas
#include "img_proc/Find_piece_Srv.h"
#include "img_proc/Align_Srv.h"

//Para encontrar aruco
#include "img_proc/Find_tag_Srv.h"

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
    SM_ALIGN,
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

std::string zone;

std::vector<std::string> real_refbox_names;
std_msgs::String new_zone;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

bool request = false;

float angulo = 135;
float angulo_rad;

//Parametro que multiplica al coseno 
float param_x = 0.5;
//Parametro que multiplica al seno
float param_y = 0.8;

//Función para ya hacer pruebas con el Refbox
void compute_coordinates(){
    if(tokens[4] == "entrance" || tokens[4] == "platform" ){
        tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + param_x*cos(std::stoi(tokens[3])*(M_PI/180));
        tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + param_y*sin(std::stoi(tokens[3])*(M_PI/180)); 
        angulo = angulo - 180;                       
    }
    if(tokens[4] == "output"){
        tf_target_zone.pose.position.x = tf_target_zone.pose.position.x - param_x*cos(std::stoi(tokens[3])*(M_PI/180));
        tf_target_zone.pose.position.y = tf_target_zone.pose.position.y - param_y*sin(std::stoi(tokens[3])*(M_PI/180));
    }
    angulo_rad = angulo*M_PI/180;

    tf::Quaternion myQuaternion;

    myQuaternion.setRPY(0,0,angulo*M_PI/180);

    myQuaternion=myQuaternion.normalize();

    tf_target_zone.pose.orientation.x = myQuaternion[0];
    tf_target_zone.pose.orientation.y = myQuaternion[1];
    tf_target_zone.pose.orientation.z = myQuaternion[2];
    tf_target_zone.pose.orientation.w = myQuaternion[3];
}

//Función hardcodeada para hacer pruebas rápidas
// void compute_coordinates(){
//     float quat;

//     //tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + 0.6*cos(std::stoi("135")*(M_PI/180));
//     //tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + 0.6*sin(std::stoi("135")*(M_PI/180));  

//     tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + param_x*cos(angulo*(M_PI/180));
//     tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + param_y*sin(angulo*(M_PI/180)); 

//     //Si es ir a la entrada entonces se obtiene el complemento del ángulo en 180
//     //Si es ir a la salida entonces se queda igual el ángulo
//     //if(tokens[4] == "entrance" || tokens[4] == "platform" ){
//         angulo = angulo - 180;                     
//     //}
// 	angulo_rad = angulo*M_PI/180;

//     std::cout << "el ángulo en grados es: " << angulo << std::endl;
//     std::cout << "el ángulo en rad es: " << angulo*M_PI/180 << std::endl;

//     tf::Quaternion myQuaternion;

//     myQuaternion.setRPY(0,0,angulo*M_PI/180);

//     myQuaternion=myQuaternion.normalize();

//     tf_target_zone.pose.orientation.x = myQuaternion[0];
//     tf_target_zone.pose.orientation.y = myQuaternion[1];
//     tf_target_zone.pose.orientation.z = myQuaternion[2];
//     tf_target_zone.pose.orientation.w = myQuaternion[3];

//     std::cout << "Coordenadas modificadas:" << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
                    
// }

//Funcion para ya hacer pruebas con el refbox
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
        listener.waitForTransform("/map", tokens.at(2), ros::Time(0), ros::Duration(1000.0));
        listener.lookupTransform("/map", tokens.at(2), ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    tf_target_zone.pose.position.x = transform.getOrigin().x();
    tf_target_zone.pose.position.y = transform.getOrigin().y();
	tf_target_zone.pose.position.z = transform.getOrigin().z();
	tf_target_zone.pose.orientation.x = transform.getRotation().x();
	tf_target_zone.pose.orientation.y = transform.getRotation().y();
	tf_target_zone.pose.orientation.z = transform.getRotation().z();
	tf_target_zone.pose.orientation.w = transform.getRotation().w();

    std::cout << "salió del try name:" << tokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
}

//Funcion hardcodeada para hacer pruebas rapidas

// void transform_zone()
// {
// 	tf::TransformListener listener;
//     tf::StampedTransform transform;

//     zone = "C_Z42";

//     //TF related stuff 
//     tf_target_zone.header.frame_id = "/map";
//     tf_target_zone.pose.position.x = 0.0;
//     tf_target_zone.pose.position.y = 0.0;
//     tf_target_zone.pose.position.z = 0.0;
//     tf_target_zone.pose.orientation.x = 0.0;
//     tf_target_zone.pose.orientation.y = 0.0;
//     tf_target_zone.pose.orientation.z = 0.0;
//     tf_target_zone.pose.orientation.w = 0.0;

//     std::cout << "entró al transform zones" << std::endl;

//     try{
//         std::cout << "entró al try" << std::endl;
//         listener.waitForTransform("/map",zone, ros::Time(0), ros::Duration(1000.0));
//         listener.lookupTransform("/map",zone, ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//         ROS_ERROR("%s",ex.what());
//         ros::Duration(1.0).sleep();
//     }

//     //tf_target_zone.pose.position.x = -transform.getOrigin().x();
//     //tf_target_zone.pose.position.y = -transform.getOrigin().y();

//     tf_target_zone.pose.position.x = transform.getOrigin().x();
//     tf_target_zone.pose.position.y = transform.getOrigin().y();
// 	tf_target_zone.pose.position.z = transform.getOrigin().z();
// 	tf_target_zone.pose.orientation.x = transform.getRotation().x();
// 	tf_target_zone.pose.orientation.y = transform.getRotation().y();
// 	tf_target_zone.pose.orientation.z = transform.getRotation().z();
// 	tf_target_zone.pose.orientation.w = transform.getRotation().w();

//     //std::cout << "salió del try name:" << tokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
//     std::cout << "salió del try name:" << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
//     std::cout << "Las rotaciones son" << " ori x:" << tf_target_zone.pose.orientation.x << std::endl;
//     std::cout << "Las rotaciones son" << " ori y:" << tf_target_zone.pose.orientation.y << std::endl;
//     std::cout << "Las rotaciones son" << " ori z:" << tf_target_zone.pose.orientation.z << std::endl;
//     std::cout << "Las rotaciones son" << " ori w:" << tf_target_zone.pose.orientation.w << std::endl;
// }

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
    std::cout << "Entré al callback de instrucciones" << msg->data.c_str() <<std::endl;	
    //Tokenize instruction string
    target_zone = *msg;
    std::cout << "La instrucción es: " <<  target_zone <<std::endl;	
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

// void callback_slope(const std_msgs::Float32::ConstPtr& msg)
// {
//     float slope = (*msg).data;

//     float th = 0.03f;

//     float Kp = -6.0f
//     float Kp_m = 6.0f

//     if(slope < th && slope > -th){
//         std::cout << "Alineado!!!" << std::endl;
//     }
//     else{
//         std::cout << "Ño alineado" << std::endl;
//         if (slope < 0 && slope != 1){
//             FestinoNavigation::moveDistAngle(0.0, Kp_m*slope, 10000);
//         }
//         else if (slope > 0 && slope != 1){
//             FestinoNavigation::moveDistAngle(0.0, Kp*abs(error), 10000);
//         }
//     }
// }



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
    //ros::Subscriber subSlope        = n.subscribe("/slope_data", 10, callback_slope);
    ros::Publisher pubRequest       = n.advertise<std_msgs::String>("/request_instruction", 1000);
    ros::Publisher pub_rosnav_goal  = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000, true);
    ros::Publisher pubMachineInst   = n.advertise<std_msgs::String>("/machine_instruction_msg", 1000);
    ros::Publisher pubManipulator   = n.advertise<std_msgs::Int32 >("manipulator/action", 1000);

    //Declarar servicio para encontrar pieza
    ros::ServiceClient piece_client 		= n.serviceClient<img_proc::Find_piece_Srv>("/vision/find_piece/point_stamped");
    ros::ServiceClient aruco_client 		= n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");

    img_proc::Find_piece_Srv piece_srv;
    img_proc::Find_tag_Srv aruco_srv;

    ros::Rate loop(30);

    std::string voice;

    //String que se le envía al planeador para pedirle una instrucción 
    std_msgs::String request_string;
    request_string.data = "Ola khe ase";

    //Entero que se le envía al nodo de la pinza 
    std_msgs::Int32 manipulator_var;

    //String que se le envía a las máquinas para pedirles cosas
    std_msgs::String machine_instruction;

    int cont = 0;

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            voice = "I am ready for the main track challenge";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,5);
	    		state = SM_WAIT_FOR_INSTRUCTION;
                //state = SM_GO_TO;
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
	            voice = "Navigating to// error = slope";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

                //A partir de la instruccion se extrae la zona y con lookTransform se encuentran las coordenadas correspondientes
                transform_zone();
                //Dependiendo de la orientacion de la maquina y de si se quiere ir a la entrada o salida se obtienen las coordenadas
                //tomando como base las coordenadas x,y de la zona, que representan el centro.
                compute_coordinates();

                //Navegacion Marco
                navigate_to_location(tf_target_zone);
		        FestinoNavigation::moveDistAngle(0.0, angulo_rad, 10000);

                //Navegacion ROS para hacer pruebas
                //pub_rosnav_goal.publish(tf_target_zone);
				ros::Duration(10, 0).sleep();

                state = SM_ALIGN;
	    		break;

            case SM_ALIGN:
                std::cout << "State machine: SM_ALIGN" << std::endl;
	            voice = "Aligning";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

                aruco_srv.request.is_find_tag_enabled = true;
				aruco_client.call(aruco_srv);
				if(aruco_srv.response.success){
                    std::cout << "Ya se alineo en angulo" << std::endl;
                    aruco_srv.request.is_find_tag_enabled = false;
                    aruco_srv.request.is_aling_enabled = true;
				    aruco_client.call(aruco_srv);

                    if(aruco_srv.response.success){
                        std::cout << "Alineado!!!" << std::endl;
                    
					    FestinoNavigation::moveDistAngle(0.50, 0, 10000);
					    state = SM_WAIT_FOR_INSTRUCTION;	
                    }
                    else{
                        std::cout << "NotFound" << std::endl;
					    state = SM_FIND;
                    }
				}
				else{
					std::cout << "NotFound" << std::endl;
					state = SM_FIND;
				}

                state = SM_FINAL_STATE;
                break;
	    	case SM_FIND:
	            std::cout << "State machine: SM_FIND" << std::endl;
	            voice = "Finding the piece";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

                piece_srv.request.is_find_piece_enabled = true;
				piece_srv.request.piece = "Red";
				piece_client.call(piece_srv);
				if(piece_srv.response.success){
					std::cout << "Found, dise" << std::endl;
					std::cout << "Resting pose" << std::endl;
					
                    //Mover brazo 

					state = SM_WAIT_FOR_INSTRUCTION;	
				}
				else{
					std::cout << "NotFound" << std::endl;
					state = SM_FIND;
				}
                
	            break;
	        
			case SM_TAKE:
	    		std::cout << "State machine: SM_TAKE" << std::endl;	
	            voice = "Grasping the piece";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
                //Reposo
                //Reposo con pieza
                //Pick
                //Place 
                manipulator_var.data = 1;
                pubManipulator.publish(manipulator_var);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_MOVE:
	    		std::cout << "State machine: SM_MOVE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_DROP:
	    		std::cout << "State machine: SM_DROP" << std::endl;	
	            voice = "Droping the piece";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                manipulator_var.data = 2;
                pubManipulator.publish(manipulator_var);

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_ASK:
	    		std::cout << "State machine: SM_ASK" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);

                //Envía la concatenada la acción y el color de la base si se trata de la BS
                machine_instruction.data = tokens[1]  + " " + tokens[2];
	    		
                pubMachineInst.publish(machine_instruction);
                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
	            voice =  "I have finished test";
	            std::cout << voice << std::endl;
				FestinoHRI::say(voice,3);
	    		
                state = SM_FINAL_STATE;
	    		break;
		}
        ros::Duration(1, 0).sleep();
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}
