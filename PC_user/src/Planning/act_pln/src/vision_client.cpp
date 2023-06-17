#include "ros/ros.h"
#include "img_proc/Find_piece_Srv.h"
//#include <cstdlib>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_client");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<img_proc::Find_piece_Srv>("/vision/find_piece/point_stamped");
  img_proc::Find_piece_Srv srv;
  srv.request.is_find_piece_enabled = true;
  bool jelp = false;
  if (client.call(srv))
  {
    while(jelp != true)
    {
      ROS_INFO("Resultado", srv.response.success);
      jelp = srv.response.success;
      std::cout << jelp  << std::endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}