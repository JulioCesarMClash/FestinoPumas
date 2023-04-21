#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include "vision_msgs/Keypoint.h"
#include "vision_msgs/HumanCoordinates.h"
#include "vision_msgs/HumanCoordinatesArray.h"

vision_msgs::HumanCoordinatesArray HumanArray;
std::vector<vision_msgs::HumanCoordinates> Human;

void keyPointsCallback(const vision_msgs::HumanCoordinatesArray::ConstPtr& msg)
{
  HumanArray = *msg;
  Human = HumanArray.coordinates_array;
  if(Human.size() > 0){
	std::cout << "Hay un humano"<< std::endl;
  }
  else{
  	std::cout << "No hay un humano" << std::endl;
  }
  //std::cout << Human.coordinates_array << std::endl;
  //std::cout << HumanArray << std::endl;
  //std::cout << typeid(HumanArray).name() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("human_coordinates_array", 1000, keyPointsCallback);
  ros::spin();

  return 0;
}
