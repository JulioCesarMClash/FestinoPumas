// Bibliotecas generales
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include <ros/ros.h>

// Bibliotecas de festino
#include <festino_tools/FestinoHRI.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoKnowledge.h>
#include <festino_tools/FestinoHardware.h>

// Estados
enum SMState
{
        SM_INIT,
        SM_WAIT_FOR_DOOR,
        SM_SAY_OPEN_DOOR,
        SM_NAVIGATE_TO_STORING_POINT,
        SM_FIND_OBJECTS,
        SM_PRE_GRASP,
        SM_TRY_TO_GRASPING_OBJECT,
        SM_NAVIGATE_TO_SHELF,
        SM_FINISH_TEST
};

// VARIABLES GLOBALES
// Banderas
bool success = false;

// Contadores

// Para el movimiento de la cabeza
float pitchAngle;

// Variables auxiliares de navegación
float robot_y, robot_x, robot_a;    
float gx_w, gy_w, gz_w, guest_z, host_z;    
float goalx, goaly, goala;
float dist_to_head;
float theta = 0, thetaToGoal = 0, angleHead = 0;
float pointingArmX, pointingArmY, pointingArmZ;
float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
float distanceArm = 0.6;

//Strings aux

std::stringstream ss;
std::stringstream ss2;

ros::NodeHandle nh;

SMState state = SM_INIT;

// Variables de inicio
std::vector<float> goal_vec(3);

int main(int argc, char **argv)
{
    std::cout << "INITIALIZING ACT_PLN BY JOSHUA M... Aaaaaaaiudaaaaa" << std::endl;
    ros::init(argc, argv, "storing_groseries_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    FestinoHardware::setNodeHandle(&nh);
    FestinoHRI::say(" ",2);

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
                case SM_INIT:
                        std::cout << "SM_INIT --> Start Storing groseries :)" << std::endl;
                        FestinoHRI::say("I'm ready for storing groseries test", 3);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_WAIT_FOR_DOOR:
                        std::cout << "SM_WAIT_FOR_DOOR --> I'm waitig for the door is open" << std::endl;
                        state = FestinoNavigation::waitForDoor() ? SM_SAY_OPEN_DOOR : SM_NAVIGATE_TO_STORING_POINT;
                        break;

                case SM_SAY_OPEN_DOOR:
                        std::cout << "SM_SAY_OPEN_DOOR --> I'm saying to human that open the door" << std::endl;
                        FestinoHRI::say("Human, please open the door,", 3);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_NAVIGATE_TO_STORING_POINT:
                        std::cout << "SM_NAVIGATE_TO_STORING_POINT --> I'm navigating to the storing point" << std::endl;
                        
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("storing_table");
                        std::cout <<"Coordenates of storing table:"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            std::cout << "Cannot move to storing table" << std::endl; 
                        
                        FestinoHRI::say("I have arrived to storing table",3);	
                        state = SM_PRE_GRASP;
                        break;

                case SM_PRE_GRASP:
                        std::cout << "SM_PRE_GRASP --> I'm moving my left arm to a pre-grasp position" << std::endl;
                        FestinoHardware::setArmPose("pre_grasp");
                        sleep(2);
                        state = SM_FINISH_TEST;
                        break;

                case SM_FIND_OBJECTS:
   
                        break;

                case SM_TRY_TO_GRASPING_OBJECT:

                        break;
                
                case SM_NAVIGATE_TO_SHELF:
                        break;
                
                case SM_FINISH_TEST:
                        std::cout << "SM_FINISH_TEST --> I finish the test: wuuuuu :)" << std::endl;
                        
                        break;
        }
    }
}