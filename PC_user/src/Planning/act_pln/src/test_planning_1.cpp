#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>


sensor_msgs::LaserScan laserScan;

ros::NodeHandle* nh;

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_CHECK_DOOR,
	SM_OPEN_DOOR,
	SM_CLOSE_DOOR,
    SM_SET_LOC,
    SM_NAVIGATE_LOC,
	SM_FINAL_STATE
};

SMState state = SM_INIT;

bool fail = false;
bool success = false;

bool flag_door = false;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;

    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    //std::cout<<laserScan.ranges.size()<<std::endl;
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
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4){ 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.5){
        flag_door = true;
        //std::cout<<"door open"<<std::endl;
    }
    else {
        flag_door = false;
        //std::cout<<"door closed"<<std::endl;
    }
    
     
    
}
/*
bool detectDoorInFront()
{
    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    std::cout<<laserScan.ranges.size()<<std::endl;
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    
    //cont_laser=0;
    //laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4){ 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    
    if((laser_l/cont_laser < 0.5) && (laserScan.ranges.size() != 0)){
        std::cout<<laserScan.ranges.size()<<std::endl;
        std::cout<<"Range Size: "<< range << "\n ";
        std::cout<<"RangInie Central: "<< range_c << "\n ";
        std::cout<<"Range tial: "<< range_i << "\n ";
        std::cout<<"Range Final: "<< range_f << "\n ";
        std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
        return true;
    }
    return false;
}
*/
int main(int argc, char** argv)
{
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "test_planning_1");
    ros::NodeHandle n;
    ros::Subscriber subLaserScan = n.subscribe("/scan", 1, callbackLaserScan);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);//, latch=True);
    ros::Rate loop(30);
    geometry_msgs::PoseStamped loc_tf;
    loc_tf.header.frame_id = "/map";
    loc_tf.pose.position.x = 0.0;
    loc_tf.pose.position.y = 0.0;
    loc_tf.pose.position.z = 0.0;
    loc_tf.pose.orientation.x = 0.0;
    loc_tf.pose.orientation.y = 0.0;
    loc_tf.pose.orientation.z = 0.0;
    loc_tf.pose.orientation.w = 0.0;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while(ros::ok() && !fail && !success){
        switch(state){

        	case SM_INIT:
        		//Init case
        		std::cout << "State machine: SM_INIT" << std::endl;	
                std::cout << "I am ready for the navigation test" << std::endl; 
        		state = SM_CHECK_DOOR;
        		break;

        	case SM_CHECK_DOOR:
        		//Checking open door case
        		std::cout << "State machine: SM_CHECK_DOOR" << std::endl;
                sleep(1);
        		if(flag_door == false){
        			state = SM_CLOSE_DOOR;	
        		}	
        		else{
        			state = SM_OPEN_DOOR;	
        		}
        		break;

        	case SM_OPEN_DOOR:{
        		//Navigate case
        		std::cout << "State machine: SM_OPEN_DOOR" << std::endl;
                try{
                      listener.lookupTransform(argv[1], "/map",  
                                               ros::Time(0), transform);
                    }
                    catch (tf::TransformException ex){
                      ROS_ERROR("%s",ex.what());
                      ros::Duration(1.0).sleep();
                    }
                std::cout << transform.getOrigin().x() << ',' << transform.getOrigin().y() << std::endl;
                loc_tf.pose.position.x = -transform.getOrigin().x();
                loc_tf.pose.position.y = -transform.getOrigin().y();
                //loc_tf.pose.position.x = 4.629;
                //loc_tf.pose.position.y = -3.458;
                pub_goal.publish(loc_tf);
                std::cout << "Goal: "<< argv[1] << "-> (" << loc_tf.pose.position.x << ", " << loc_tf.pose.position.y << ")\n" << std::endl;
        		state = SM_FINAL_STATE;
        		break;
            }

        	case SM_CLOSE_DOOR:{
        		//door closed case
        		std::cout << "State machine: SM_CLOSE_DOOR" << std::endl;
        		std::cout << "The door is closed" << std::endl;	
        		state = SM_CHECK_DOOR;
        		break;
            }

        	case SM_FINAL_STATE:
        		//Navigate case
        		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
        		sleep(3);
        		std::cout << "I have finished test" <<std::endl;
        		success = true;
        		fail = true;
        		break;


        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;

}