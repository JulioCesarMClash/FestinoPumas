#include <grips_simulation/grips_robot_plugin.h>
#include <iostream>
#include <math.h>
#include <limits>

#define SEARCH_AREA_REL_X 0.7
#define SEARCH_AREA_REL_Y 0.3
#define RADIUS_DETECTION_AREA 0.35

using namespace ignition;

GripsRobotPlugin::~GripsRobotPlugin()
{
  ROS_INFO("Destructing GripsRobotPlugin!");
  alive_ = false;
  gripper_opc_client_->Disconnect();
}

void GripsRobotPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  //visible_ = false;

  state_red_ = llsf_msgs::OFF;
  state_yellow_ = llsf_msgs::OFF;
  state_green_ = llsf_msgs::OFF;

  std::cout << "GripsRobotPlugin::Load" << std::endl;

  this->parent_ = _parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "GripsRobot"));

  std::cout << "name = " << parent_->GetName() << std::endl;

  gazebo_ros_->isInitialized();

  vx_ = 0.0;
  vy_ = 0.0;
  vomega_ = 0.0;

  alive_ = true;

  this->name_ = parent_->GetName();

  last_update_time_ = parent_->GetWorld()->SimTime();

  global_loc_ = true; // false; testing!

  if (_sdf->HasElement("globalLocalization") && !_sdf->Get<std::string>("globalLocalization").compare("true"))
  {
    std::cout << "globalLocalization activated" << std::endl;
    global_loc_ = true;
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    tf_prefix = _sdf->Get<std::string>("robotNamespace");
  }
  else
  {
    ROS_ERROR("grips_robot_plugin: tf_prefix not found, cannot load params for gripper OPC connection!");
  }

  this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node_->Init(parent_->GetWorld()->Name() + "/" + name_);

  gripper_pub_ = this->node_->Advertise<gazebo::msgs::Int>("~/RobotinoSim/SetGripper/");
  gripper_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/GripperHasPuck/"), &GripsRobotPlugin::hasPuckCB, this);
  has_puck_ = false;

  std::string opc_address;

  opc_address = update_opc_server_address(_sdf);

  if (!opc_address.empty())
  {
    ROS_INFO_STREAM("Starting OPC connection to " + opc_address);
    gripper_opc_client_ = std::shared_ptr<GripperInterface::OpcUaClient>(
        new GripperInterface::OpcUaClient(opc_address, GripperInterface::Logger::RosLogger));
    gripper_opc_client_->StartConnection();
    opc_connect_thread_ = boost::thread(boost::bind(&GripsRobotPlugin::waitForOpcAndSubscribe, this));
  }
  

  light_msg_sub_ = this->node_->Subscribe(std::string("/gazebo/LLSF/LLSFRbSim/MachineInfo/"),
                                          &GripsRobotPlugin::on_light_msg, this);

  odometry_frame_ = "odom";
  robot_base_frame_ = "base_link";
  amcl_frame_ = "map";

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>("cmd_vel", 1,
                                                          boost::bind(&GripsRobotPlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &cmd_vel_queue_);

  cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);

  update_period_ = 0.02;

  std::cout << "GripsRobotPlugin Odometry init" << std::endl;

  odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>("odom", 1);
  if (global_loc_)
  {
    pose_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
  }

  transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  std::cout << "GripsRobotPlugin AMCL init" << std::endl;

  this->cmd_vel_callback_queue_thread_ =
      boost::thread(boost::bind(&GripsRobotPlugin::CmdVelQueueThread, this));
  this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&GripsRobotPlugin::UpdateChild, this));

  axis_x_name_ = parent_->GetName() + "::gripper::axis_x";
  axis_y_name_ = parent_->GetName() + "::gripper::axis_y";

  controller_ = parent_->GetJointController();

  std::map<std::string, gazebo::common::PID> cpid = controller_->GetPositionPIDs();

  for (std::map<std::string, gazebo::common::PID>::iterator it = cpid.begin(); it != cpid.end(); it++)
  {
    controller_->SetPositionPID(it->first, gazebo::common::PID(1000, 0, 100, 0, 0, 1000, -1000));
  }

  controller_->SetPositionPID(axis_x_name_, gazebo::common::PID(2000, 0, 100, 0, 0, 2000, -2000));
  controller_->SetPositionPID(axis_y_name_, gazebo::common::PID(2000, 0, 100, 0, 0, 2000, -2000));
  controller_->SetPositionTarget(axis_x_name_, 0.0);
  controller_->SetPositionTarget(axis_y_name_, 0.0);
  controller_->Update();
}

std::string GripsRobotPlugin::update_opc_server_address(sdf::ElementPtr _sdf)
{
  std::string opc_address;

    while (!gazebo_ros_->node()->getParam("/" + tf_prefix + "/gripper_interface/opc_server_address", opc_address))
    {
      ROS_WARN("grips_robot_plugin: opc_server_address not yet configured, waiting for it...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    ROS_INFO_STREAM("grips_robot_plugin: opc_server_address configured: " << opc_address);
  return opc_address;
}

void GripsRobotPlugin::CmdVelQueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && gazebo_ros_->node()->ok())
  {
    cmd_vel_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GripsRobotPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg)
{
  //ROS_INFO("cmdVelCallback ...");
  vx_ = cmd_msg->linear.x;
  vy_ = cmd_msg->linear.y;
  vomega_ = cmd_msg->angular.z;
}

void GripsRobotPlugin::UpdateChild()
{
  // std::cout << "GripsRobotPlugin::UpdateChild()" << std::endl;
  robot_pose_ = parent_->WorldPose();

  gazebo::common::Time current_time = parent_->GetWorld()->SimTime();

  float yaw = this->parent_->WorldPose().Rot().Euler().Z();

  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update >= update_period_)
  {
    publishOdometry(seconds_since_last_update);
    last_update_time_ = current_time;
  }
  this->parent_->SetLinearVel(
      math::Vector3d(vx_ * cos(yaw) + vy_ * cos(yaw + M_PI / 2.0), vx_ * sin(yaw) + vy_ * sin(yaw + M_PI / 2.0),
                     0));
  this->parent_->SetAngularVel(math::Vector3d(0, 0, vomega_));

  std::map<std::string, gazebo::common::PID> cpid = controller_->GetPositionPIDs();

  /*  for(std::map<std::string, common::PID>::iterator it = cpid.begin(); it != cpid.end();it++)
  {
    double d, i, p;
    it->second.GetErrors(d, i, p);
    std::cout << it->first << " " << it->second.GetCmd() << " " << it->second.GetCmdMax() << " " << it->second.GetPGain() << " error= " << d << std::endl;
  } */
}

void GripsRobotPlugin::on_light_msg(ConstMachineInfoPtr &msg)
{

  //Calculate Robot detection center
  double look_pos_x = robot_pose_.Pos().X() + cos(robot_pose_.Rot().Yaw()) * SEARCH_AREA_REL_X -
                      sin(robot_pose_.Rot().Yaw()) * SEARCH_AREA_REL_Y;
  double look_pos_y = robot_pose_.Pos().Y() + sin(robot_pose_.Rot().Yaw()) * SEARCH_AREA_REL_X +
                      cos(robot_pose_.Rot().Yaw()) * SEARCH_AREA_REL_Y;

  //std::cout << "Robot center is at: (x,y) = (" << robot_pose_.pos.x << "," << robot_pose_.pos.y << ")" << std::endl;
  //std::cout << "Robot detection center is at: (x,y) = (" << look_pos_x << "," << look_pos_y << ")" << std::endl;

  // find mearest machine in front of the robot
  int nearest_index = -1;
  float min_dist = std::numeric_limits<float>::infinity();
  for (int i = 0; i < msg->machines_size(); i++)
  {
    llsf_msgs::Machine machine = msg->machines(i);
    std::string machine_name = machine.name();

    std::string light_link_name = machine_name + "::light_signals::link";
    physics::EntityPtr light_entity = parent_->GetWorld()->EntityByName(light_link_name.c_str());
    if (light_entity == NULL)
    {
      //printf("Light-Signal-Detection can't find machine with name %s!\n", machine_name.c_str());
      return;
    }
    auto light_pose = light_entity->WorldPose();
    //std::cout << "light_pose is (x,y,z): (" << light_pose.pos.x << "," << light_pose.pos.y << "," << light_pose.pos.z << ")" << "\t";
    float dist = light_pose.Pos().Distance(look_pos_x, look_pos_y, light_pose.Pos().Z());

    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_index = i;
    }
  }
  //std::cout << "Found nearest dist is " << min_dist << " at index " << nearest_index << "." << std::endl;
  if (min_dist < RADIUS_DETECTION_AREA)
  {
    save_light_signal(msg->machines(nearest_index));
  }
  else
  {
    clear_light_signal();
  }
}

void GripsRobotPlugin::save_light_signal(llsf_msgs::Machine machine)
{
  //go through all light specs
  //set default values
  clear_light_signal();
  for (int i = 0; i < machine.lights_size(); i++)
  {
    llsf_msgs::LightSpec light_msg = machine.lights(i);
    switch (light_msg.color())
    {
    case llsf_msgs::RED:
      state_red_ = light_msg.state();
      break;
    case llsf_msgs::YELLOW:
      state_yellow_ = light_msg.state();
      break;
    case llsf_msgs::GREEN:
      state_green_ = light_msg.state();
      break;
    }
  }
}

void GripsRobotPlugin::clear_light_signal()
{
  //go through all light specs
  //set default values
  state_red_ = llsf_msgs::OFF;
  state_yellow_ = llsf_msgs::OFF;
  state_green_ = llsf_msgs::OFF;
}

void GripsRobotPlugin::publishOdometry(double step_time)
{
  //std::cout << "GripsRobotPlugin::publishOdometry()" << step_time  << std::endl;

  gazebo::common::Time gz = parent_->GetWorld()->SimTime();
  ros::Time current_time = ros::Time(gz.Double(), (gz.Double() - floor(gz.Double())) * 1E9);

  //ros::Time current_time = ros::Time::now();
  std::string odom_frame = gazebo_ros_->resolveTF(tf_prefix + "/" + odometry_frame_);
  std::string base_footprint_frame = gazebo_ros_->resolveTF(tf_prefix + "/" + robot_base_frame_);
  //std::string amcl_frame = gazebo_ros_->resolveTF(amcl_frame_);
  std::string amcl_frame = "map";
  std::string laser_frame = gazebo_ros_->resolveTF(tf_prefix + "/laser");
  std::string camera_ar_frame = gazebo_ros_->resolveTF(tf_prefix + "/camera_ar_link");
  std::string prod_cam_frame = gazebo_ros_->resolveTF(tf_prefix + "/prod_cam_link");

  tf::Quaternion qt;
  tf::Vector3 vt;

  // getting data form gazebo world
  math::Pose3d pose = parent_->WorldPose();
  qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  pose_.pose.pose.position.x = vt.x();
  pose_.pose.pose.position.y = vt.y();
  pose_.pose.pose.position.z = vt.z();

  pose_.pose.pose.orientation.x = qt.x();
  pose_.pose.pose.orientation.y = qt.y();
  pose_.pose.pose.orientation.z = qt.z();
  pose_.pose.pose.orientation.w = qt.w();

  // get velocity in /odom frame
  auto linear = parent_->WorldLinearVel();
  odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();

  // convert velocity to child_frame_id (aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(
      tf::StampedTransform(base_footprint_to_odom, current_time,
                           odom_frame, base_footprint_frame));

  vt.setZero();

  if (global_loc_)
  {
    tf::Transform odom_to_amcl(tf::createIdentityQuaternion(), vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(odom_to_amcl, current_time,
                             amcl_frame, odom_frame));
  }

  vt = tf::Vector3(0.17, 0, 0.22);

  tf::Transform laser_to_base(tf::createIdentityQuaternion(), vt);
  transform_broadcaster_->sendTransform(
      tf::StampedTransform(laser_to_base, current_time,
                           base_footprint_frame, laser_frame));

  vt = tf::Vector3(0.2, 0, 0.53);

  tf::Transform camera_ar_to_base(tf::createQuaternionFromRPY(1.571, 3.1415, 1.571), vt);
  transform_broadcaster_->sendTransform(
      tf::StampedTransform(camera_ar_to_base, current_time,
                           base_footprint_frame, camera_ar_frame));
  vt = tf::Vector3(0.1, 0, 1.06);
  tf::Transform prod_cam_to_base(tf::createQuaternionFromRPY(1.1, 3.1415, 1.571), vt);

  transform_broadcaster_->sendTransform(
      tf::StampedTransform(prod_cam_to_base, current_time,
                           base_footprint_frame, prod_cam_frame));

  // set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  pose_.pose.covariance[0] = 0.00001;
  pose_.pose.covariance[7] = 0.00001;
  pose_.pose.covariance[14] = 1000000000000.0;
  pose_.pose.covariance[21] = 1000000000000.0;
  pose_.pose.covariance[28] = 1000000000000.0;
  pose_.pose.covariance[35] = 0.001;

  // set header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  pose_.header.stamp = current_time;
  pose_.header.frame_id = amcl_frame;

  odometry_publisher_.publish(odom_);

  if (global_loc_)
  {
    pose_publisher_.publish(pose_);
  }
}

void GripsRobotPlugin::hasPuckCB(ConstIntPtr &msg)
{
  has_puck_ = msg->data();
}

void GripsRobotPlugin::waitForOpcAndSubscribe()
{
  // disable opc log until connected to prevent log spamming
  gripper_opc_client_->Log.Disable();
  if (!gripper_opc_client_->WaitForConnection(std::chrono::milliseconds(300000)))
  {
    ROS_ERROR("Could not connect to gripper OPC server within 300 seconds!");
    return;
  }
  gripper_opc_client_->Log.Enable();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  uint32_t gripperClosedHandle, rAxisHandle, phiAxisHandle;
  boost::function<void(uint32_t, std::string, bool)> gripperClosedCB(boost::bind(&GripsRobotPlugin::gripperClosedSubCallback, this, _1, _2, _3));
  boost::function<void(uint32_t, std::string, double)> rAxisCB(boost::bind(&GripsRobotPlugin::rAxisSubCallback, this, _1, _2, _3));
  boost::function<void(uint32_t, std::string, double)> phiAxisCB(boost::bind(&GripsRobotPlugin::phiAxisSubCallback, this, _1, _2, _3));

  opcSubscribeWithRetry(GripperInterface::OpcNodesFromPLC::GripperClosed, gripperClosedCB, gripperClosedHandle);
  opcSubscribeWithRetry(GripperInterface::OpcNodesFromPLC::PositionR, rAxisCB, rAxisHandle);
  opcSubscribeWithRetry(GripperInterface::OpcNodesFromPLC::PositionPhi, phiAxisCB, phiAxisHandle);

  gripper_axis_positioning_thread = boost::thread(boost::bind(&GripsRobotPlugin::GripperAxisPositioningThread, this));
}

void GripsRobotPlugin::gripperClosedSubCallback(uint32_t handle, std::string nodeId, bool value)
{
  gazebo::msgs::Int cmd;
  if (value)
  {
    ROS_INFO("Close Gripper");
    cmd.set_data(0);
  }
  else
  {
    ROS_INFO("Open Gripper");
    cmd.set_data(1);
  }
  gripper_pub_->Publish(cmd);
  ros::Duration(0.01).sleep();
}

void GripsRobotPlugin::rAxisSubCallback(uint32_t handle, std::string nodeId, double value)
{
  if (value < 0)
  {
    ROS_ERROR_STREAM("Gripper x-Axis value " << value << " not valid");
    return;
  }
  if (value > 0.1)
    rAxisTarget_ = 0.1;
  else
    rAxisTarget_ = value;
}

void GripsRobotPlugin::phiAxisSubCallback(uint32_t handle, std::string nodeId, double value)
{
  if (value < -0.3)
    phiAxisTarget_ = -0.3;
  else if (value > 0.1)
    phiAxisTarget_ = 0.1;
  else
    phiAxisTarget_ = value;
}

void GripsRobotPlugin::GripperAxisPositioningThread()
{
  while (alive_ && gazebo_ros_->node()->ok())
  {
    if (fabs(rAxisPosition_ - rAxisTarget_) > 1e-3 || fabs(phiAxisPosition_ - phiAxisTarget_) > 1e-3)
    {
      controller_->SetPositionTarget(axis_x_name_, rAxisTarget_);
      controller_->SetPositionTarget(axis_y_name_, phiAxisTarget_);

      double xerr, yerr, ie, de;
      ros::Duration(0.05).sleep();
      int counter = 0;
      do
      {
        rAxisPosition_ = controller_->GetPositions()[axis_x_name_];
        phiAxisPosition_ = controller_->GetPositions()[axis_y_name_];
        controller_->GetPositionPIDs()[axis_x_name_].GetErrors(xerr, ie, de);
        controller_->GetPositionPIDs()[axis_y_name_].GetErrors(yerr, ie, de);
        ros::Duration(0.1).sleep();
        counter++;
      } while (((fabs(xerr)) > 1e-3 || fabs(yerr > 1e-3)) && counter < 50);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GripsRobotPlugin)
