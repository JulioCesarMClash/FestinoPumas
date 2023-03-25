#pragma once

#include <gazebo-11/gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo-11/gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
using namespace ignition;
using namespace gazebo;

class GAZEBO_VISIBLE RobotMovePlugin : public ModelPlugin 
{
    public:
        RobotMovePlugin();
        ~RobotMovePlugin();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void cmdVelCB(const geometry_msgs::Twist::ConstPtr &cmd_msg);

        void CmdVelQueueThread();

        ros::Subscriber cmd_vel_subscriber;

    protected:
        virtual void UpdateChild();

    private:
        std::string m_name;
        physics::ModelPtr m_parent;
        ros::CallbackQueue m_cmd_vel_queue;
        boost::thread m_cmd_vel_cb_thread;
        GazeboRosPtr m_gazebo;
        event::ConnectionPtr m_update;
        double m_vx;
        double m_vy;
        double m_omega;
};