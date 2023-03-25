#ifndef GRIPS_ROBOT_PLUGIN_
#define GRIPS_ROBOT_PLUGIN_

#include <gazebo-11/gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo-11/gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/server/simple_action_server.h>

#include <grips_protobuf_msgs/MachineInfo.pb.h>

#include <grips_common/gripper_definitions.h>
#include <gripper_interface/opc_ua_client.h>

typedef const boost::shared_ptr<llsf_msgs::MachineInfo const> ConstMachineInfoPtr;

using namespace gazebo;
typedef const boost::shared_ptr<gazebo::msgs::Int const> ConstIntPtr;

class GripsRobotPlugin : public ModelPlugin
{
public:
  GripsRobotPlugin() {}
  ~GripsRobotPlugin();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg);

  void publishOdometry(double step_time);

  std::string update_opc_server_address(sdf::ElementPtr _sdf);
  std::string tf_prefix;
  void clear_light_signal();
  void save_light_signal(llsf_msgs::Machine machine);

  void on_light_msg(ConstMachineInfoPtr &msg);

  void hasPuckCB(ConstIntPtr &msg);

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;

  gazebo::transport::NodePtr node_;

  ros::CallbackQueue cmd_vel_queue_;
  boost::thread cmd_vel_callback_queue_thread_;
  void CmdVelQueueThread();

  ros::Subscriber cmd_vel_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher pose_publisher_;

  nav_msgs::Odometry odom_;
  geometry_msgs::PoseWithCovarianceStamped pose_;

  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

  bool alive_;

  double vx_;
  double vy_;
  double vomega_;

  llsf_msgs::LightState state_red_, state_yellow_, state_green_;

  ignition::math::Pose3d robot_pose_;

  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  std::string amcl_frame_;

  std::string name_;

  gazebo::transport::PublisherPtr gripper_pub_;
  gazebo::transport::SubscriberPtr gripper_sub_;
  gazebo::transport::SubscriberPtr light_msg_sub_;

  double update_rate_;
  double update_period_;
  gazebo::common::Time last_update_time_;

  //bool visible_;
  //int visibility_history_;
  //double visible_since_;

  bool global_loc_;

  bool has_puck_;
  std::string axis_x_name_;
  std::string axis_y_name_;

  std::shared_ptr<GripperInterface::OpcUaClient> gripper_opc_client_;
  boost::thread opc_connect_thread_;
  template <typename T>
  void opcSubscribeWithRetry(std::string nodeId, boost::function<void(uint32_t, std::string, T)> callback, uint &handle);
  void waitForOpcAndSubscribe();
  void gripperClosedSubCallback(uint32_t handle, std::string nodeId, bool value);
  void rAxisSubCallback(uint32_t handle, std::string nodeId, double value);
  void phiAxisSubCallback(uint32_t handle, std::string nodeId, double value);

  double rAxisPosition_;
  double rAxisTarget_;
  double phiAxisPosition_;
  double phiAxisTarget_;
  boost::thread gripper_axis_positioning_thread;
  void GripperAxisPositioningThread();

  gazebo::physics::JointControllerPtr controller_;
};

template <typename T>
void GripsRobotPlugin::opcSubscribeWithRetry(std::string nodeId, boost::function<void(uint32_t, std::string, T)> callback, uint &handle)
{
  int retries = 0, maxRetries = 3;
  while (retries < maxRetries && !gripper_opc_client_->SubscribeToNode(GripperInterface::OpcNamespaceId, nodeId, callback, handle))
  {
    ROS_WARN_STREAM("Could not subscribe to node " << nodeId << ", retry " << retries + 1 << "/3");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

#endif //GRIPS_ROBOT_PLUGIN_
