#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include "ros/time.h"
#include <algorithm>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

bool fail = false;
bool success = false;
bool flag_door = true;

float p_2_discard = 100;

sensor_msgs::LaserScan laserScan;

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

    for(int i = 0; i < p_2_discard; i++)
    {
    	laserScan.ranges[i] = 10.0;
    }
    for(int i = range - p_2_discard; i < range;i++)
    {
    	laserScan.ranges[i] = 10.0;
    }

    //std::cout<< laserScan << "\n ";
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

int main(int argc, char** argv){
	ros::Time::init();

	std::cout << "Fake Hokuyo... " << std::endl;
    ros::init(argc, argv, "Hokuyo_Cleaner");
    ros::NodeHandle n;

    ros::Subscriber subLaserScan 			= n.subscribe("/scan", 1, callbackLaserScan);
    ros::Publisher pub_fake_scan	= n.advertise<sensor_msgs::LaserScan>("/f_scan", 1000);

    ros::Rate loop(10);

	while(ros::ok() && !fail && !success){
	    
		pub_fake_scan.publish(laserScan);
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}