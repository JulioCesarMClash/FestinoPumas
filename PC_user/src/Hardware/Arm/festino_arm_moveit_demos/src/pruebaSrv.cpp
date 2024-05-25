#include <ros/ros.h>
#include <math.h>


// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <festino_arm_moveit_demos/srv_arm.h>

#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"



void move_effector(float x, float y, float z, float pitch);
void move_gripper(bool state);
void kinect_hide(bool flag);
bool callback_arm(festino_arm_moveit_demos::srv_arm::Request &req, festino_arm_moveit_demos::srv_arm::Response &res);

//Variables for inverse kinematics
float x = 0.23;
float y = 0.0;
float z = 0.025;
float pitch = 0;

//Variables for gripper
bool gripperState = false;
bool manipBlocker = false;
bool kinect_flag = false;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "srv_arm");
	std::cout << "pruebaSrv_node INITIALIZING" << std::endl;
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("srv_arm", callback_arm);
	ros::WallDuration(1.0).sleep();
	ros::spin();
	return 0;
}

void kinect_hide()
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setPlanningTime(4.0);
	group.setJointValueTarget("arm_shoulder_pan_joint", 2.40);
	group.setJointValueTarget("arm_shoulder_lift_joint", 1.32);
	group.setJointValueTarget("arm_elbow_flex_joint", 0.31);
	group.setJointValueTarget("arm_wrist_flex_joint", 0.61);
	group.asyncMove();
}

void move_effector(float x, float y, float z, float pitch)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setPlanningTime(4.0);

	std::cout<<"x = "<<x<<"; y = " <<y<<"; z = "<<z<< "; pitch = " <<pitch<<" ;"<<std::endl;

	tf2::Quaternion orientation;
	geometry_msgs::PoseStamped target_pose1;
	orientation.setRPY(0, pitch , atan2(y,x));

	geometry_msgs::PoseStamped det_mps;

	target_pose1.header.frame_id = "arm_base_link";
	target_pose1.pose.position.x = x;
	target_pose1.pose.position.y = y;
	target_pose1.pose.position.z = z;
	target_pose1.pose.orientation = tf2::toMsg(orientation);

	group.setPoseTarget(target_pose1);
	group.asyncMove();

	sleep(2.0);

	/*moveit::planning_interface::MoveGroupInterface group("gripper");
	group.setPlanningTime(4.0);
	group.setJointValueTarget("gripper_finger1_joint", 0.015);
	group.setJointValueTarget("gripper_finger2_joint", 0.015);
	group.asyncMove();*/
}

void move_gripper(bool state)
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("gripper");
	group.setPlanningTime(4.0);

	if(state == true) //Open
	{
		group.setJointValueTarget("gripper_finger1_joint", 0.016);
		group.setJointValueTarget("gripper_finger2_joint", 0.016);

		group.asyncMove();
	}
	if(state == false) //Close
	{
		group.setJointValueTarget("gripper_finger1_joint", 0.009);
		group.setJointValueTarget("gripper_finger2_joint", 0.009);

		group.asyncMove();
	}
}

bool callback_arm(festino_arm_moveit_demos::srv_arm::Request &req, festino_arm_moveit_demos::srv_arm::Response &res)
{
	res.success = false;
	x = req.x;
	y = req.y;
	z = req.z;
	pitch = req.pitch;

	gripperState = req.gripperState;
	manipBlocker = req.manipBlocker;
	kinect_flag  = req.kinect_hide;


	if (manipBlocker == true)
	{
		move_gripper(gripperState);
	}

	if(manipBlocker == false)
	{
		if(kinect_flag == true)
		{
			kinect_hide();
		}
		else{
			move_effector(x,y,z,pitch);
		}
	}

	res.success = true;
	return true;
}

// For z, the 0 value ref is: 0.06
// the max value is: 0.357 when Pitch = -pi/2
// -----------------------------------
// For x and y, the max value is 0.296 when the arm is complety open
// the min value is 0 when z = 0.12
