#include <ros/ros.h>
#include <math.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void move_effector (float x, float y, float z, float pitch, moveit::planning_interface::MoveGroupInterface& group);

/*void openGripper(trajectory_msgs::JointTrajectory& posture)
{

   Add both finger joints of the gripper. 
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_finger1_joint";
  posture.joint_names[1] = "gripper_finger2_joint";

  /* Set them as open, wide enough for the object to fit. 
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.015;
  posture.points[0].positions[1] = 0.015;
  posture.points[0].time_from_start = ros::Duration(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of the gripper. 
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_finger1_joint";
  posture.joint_names[1] = "gripper_finger2_joint";

  /* Set them as closed. 
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.001;
  posture.points[0].positions[1] = 0.001;
  posture.points[0].time_from_start = ros::Duration(5.0);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of gripper_fake_link. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"gripper_fake_link"` You will have to compensate for the
  // transform from `"gripper_fake_link"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "arm_base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.15;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.25;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id 
  grasps[0].pre_grasp_approach.direction.header.frame_id = "arm_base_link";
  /* Direction is set as positive x axis 
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.01;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id 
  grasps[0].post_grasp_retreat.direction.header.frame_id = "arm_base_link";
  /* Direction is set as positive z axis 
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.01;
  grasps[0].post_grasp_retreat.desired_distance = 0.02;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  //move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  //move_group.pick("object", grasps);
  move_group.move();
}*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pincher_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");

  group.setPlanningTime(4.0);

  move_effector(0.18, 0.03, -0.025, M_PI/2, group);
  geometry_msgs::Pose gripper1;
  geometry_msgs::Pose gripper2;

  gripper1.position.y = 0.015;
  gripper2.position.y = 0.015;

  group.setJointValueTarget(gripper1,"gripper_finger1_link");
  group.setJointValueTarget(gripper2,"gripper_finger2_link");

  group.move();
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}


void move_effector (float x, float y, float z, float pitch, moveit::planning_interface::MoveGroupInterface& group)
{
  tf2::Quaternion orientation;
  geometry_msgs::Pose target_pose1;

  orientation.setRPY(0, pitch , atan2(y,x));

  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  target_pose1.orientation = tf2::toMsg(orientation);

  group.setPoseTarget(target_pose1);
  group.move();
}

// For z, the 0 value ref is: 0.06
// the max value is: 0.357 when Pitch = -pi/2
// -----------------------------------
// For x and y, the max value is 0.296 when the arm is complety open
// the min value is 0 when z = 0.12