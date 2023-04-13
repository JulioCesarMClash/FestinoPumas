#include "festino_tools/FestinoKnowledge.h"

bool FestinoKnowledge::is_node_set = false;

//know_locations_parser
std::vector<float> FestinoKnowledge::_position(3);
std::vector<float> FestinoKnowledge::_orientation(4);
std::vector<float> FestinoKnowledge::_location(3);
ros::Publisher FestinoKnowledge::pubLocationParser;
ros::Subscriber FestinoKnowledge::subLocationPose;

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoKnowledge::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoKnowledge::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoKnowledge.->Setting ros node..." << std::endl;
    
    //Leg Finder
    pubLocationParser = nh->advertise<std_msgs::String>("/goal_location", 1000);
    subLocationPose = nh->subscribe("/known_location/goal", 1, &FestinoKnowledge::callbackLocPose);

    
    return true;
}

void FestinoKnowledge::GoToLocation(std::string location)
{
    std::cout << "FestinoKnowledge.->Set location: " << location << std::endl;
    std_msgs::String loc;
    loc.data = location;
    FestinoKnowledge::pubLocationParser.publish(loc);
    ros::Duration(2, 0).sleep();
}


void FestinoKnowledge::callbackLocPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	//position
	_position[0] = msg->pose.position.x;
	_position[1] = msg->pose.position.y;
	_position[2] = msg->pose.position.z;

	//orientation
	_orientation[0] = msg->pose.orientation.x;
	_orientation[1] = msg->pose.orientation.y;
	_orientation[2] = msg->pose.orientation.z;
	_orientation[3] = msg->pose.orientation.w;
}

std::vector<float> FestinoKnowledge::CoordenatesLoc()
{
	_location[0] = _position[0];
	return _location;
}