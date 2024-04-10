#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include "vision_msgs/Keypoint.h"
#include "vision_msgs/HumanCoordinates.h"
#include "vision_msgs/HumanCoordinatesArray.h"

vision_msgs::HumanCoordinatesArray HumanArray;
std::vector<vision_msgs::HumanCoordinates> Humans;
vision_msgs::HumanCoordinates Human; 
std_msgs::Bool human_bool;

void keyPointsCallback(const vision_msgs::HumanCoordinatesArray::ConstPtr& msg)
{
  HumanArray = *msg;
  //Humans = HumanArray.coordinates_array;
  //Human = HumanArray.coordinates_array[0];
  //std::cout << "Numero de Personas \t " << HumanArray.number_of_people << std::endl;
  if(HumanArray.number_of_people > 0){
    //if(Humans.size() > 0){
  	//std::cout << "Hay un humano"<< std::endl;
  	human_bool.data = true;
    //std::cout << human_bool << std::endl;
  }
  else{
  	////std::cout << "No hay un humano" << std::endl;
  	human_bool.data = false;
    //std::cout << human_bool.data << std::endl;
  }
  //std::cout << Human.coordinates_array << std::endl;
  //std::cout << HumanArray << std::endl;
  //std::cout << typeid(HumanArray).name() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detector");
  std::cout << "Human Detector node" << std::endl;
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("human_coordinates_array", 1000, keyPointsCallback);
  ros::Publisher  pub = n.advertise<std_msgs::Bool>("human_detector_bool", 1);
  ros::Rate loop(30);

  while (ros::ok())
  {
    //std::cout << "ola ke ase"<< std::endl;
    //std::cout << human_bool.data << std::endl;
  	pub.publish(human_bool);
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
