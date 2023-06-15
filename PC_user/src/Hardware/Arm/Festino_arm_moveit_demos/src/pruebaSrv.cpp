#include <ros/ros.h>
#include <math.h>


// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <Festino_arm_moveit_demos/arm.h>


void move_effector(float x, float y, float z, float pitch);
bool callback_arm(Festino_arm_moveit_demos::arm::Request &req, Festino_arm_moveit_demos::arm::Response &res);

float x = 0.10;
float y = 0.0;
float z = 0.15;
float pitch = 0;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "srv_arm_node");
	std::cout << "pruebaSrv_node INITIALIZING" << std::endl;
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("srv_arm", callback_arm);
	ros::WallDuration(1.0).sleep();
	ros::spin();
	return 0;
}

void move_effector(float x, float y, float z, float pitch)
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setPlanningTime(4.0);

	std::cout<<"x = "<<x<<"; y = " <<y<<"; z = "<<z<< "; pitch = " <<pitch<<" ;"<<std::endl;
	tf2::Quaternion orientation;
	geometry_msgs::Pose target_pose1;

	orientation.setRPY(0, pitch , atan2(y,x));

	target_pose1.position.x = x;
	target_pose1.position.y = y;
	target_pose1.position.z = z;
	target_pose1.orientation = tf2::toMsg(orientation);

	group.setPoseTarget(target_pose1);

	group.asyncMove();
}

bool callback_arm(Festino_arm_moveit_demos::arm::Request &req, Festino_arm_moveit_demos::arm::Response &res)
{
	x = req.x;
	y = req.y;
	z = req.z;
	pitch = req.pitch;
	move_effector(x,y,z,pitch);
	res.success = true;
	
	return true;
}

// For z, the 0 value ref is: 0.06
// the max value is: 0.357 when Pitch = -pi/2
// -----------------------------------
// For x and y, the max value is 0.296 when the arm is complety open
// the min value is 0 when z = 0.12
