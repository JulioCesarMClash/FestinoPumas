<<<<<<< HEAD
#include <gazebo/common/Plugin.hh>
#include <gazebo-11/gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo-11/gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <grips_simulation/robot_move_plugin.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
using namespace ignition;
using namespace gazebo;
typedef const boost::shared_ptr<gazebo::msgs::Int const> ConstIntPtr;
        RobotMovePlugin::~RobotMovePlugin() {}        

        RobotMovePlugin::RobotMovePlugin() {}

        void RobotMovePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            m_parent = _parent;
            m_name = m_parent->GetName();
            m_gazebo = GazeboRosPtr(new GazeboRos(_parent, _sdf, "GripsRobot"));

            m_gazebo->isInitialized();
=======
#include <gazebo-11/gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo-11/gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>

using namespace ignition;
using namespace gazebo;
typedef const boost::shared_ptr<gazebo::msgs::Int const> ConstIntPtr;
class RobotMovePlugin : public ModelPlugin 
{
    public:
        RobotMovePlugin() {}
        ~RobotMovePlugin();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->m_parent = _parent;
            this->m_name = this->m_parent->GetName();
>>>>>>> ff2589d8822fa20fecd780863cbd53ee42c486cf

            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<geometry_msgs::Twist>("cmd_vel", 1,
                                                          boost::bind(&RobotMovePlugin::cmdVelCB, this, _1),
                                                          ros::VoidPtr(), &m_cmd_vel_queue);
<<<<<<< HEAD
            cmd_vel_subscriber = m_gazebo->node()->subscribe(so);

            m_cmd_vel_cb_thread = 
                boost::thread(boost::bind(&RobotMovePlugin::CmdVelQueueThread, this));
            m_update =
=======

            this->m_cmd_vel_cb_thread = 
                boost::thread(boost::bind(&RobotMovePlugin::CmdVelQueueThread, this));
            this->m_update =
>>>>>>> ff2589d8822fa20fecd780863cbd53ee42c486cf
                event::Events::ConnectWorldUpdateBegin(boost::bind(&RobotMovePlugin::UpdateChild, this));

        }

<<<<<<< HEAD
        void RobotMovePlugin::cmdVelCB(const geometry_msgs::Twist::ConstPtr &cmd_msg) 
        {
            m_vx = cmd_msg->linear.x;
            m_vy = cmd_msg->linear.y;
            m_omega = cmd_msg->angular.z;
        }

        void RobotMovePlugin::CmdVelQueueThread()
        {
            static const double timeout = 0.01;

            while (m_gazebo->node()->ok())
            {
                m_cmd_vel_queue.callAvailable(ros::WallDuration(timeout));
            }
        }
        void RobotMovePlugin::UpdateChild()
        {
            float yaw = m_parent->WorldPose().Rot().Euler().Z();
            m_parent->SetLinearVel(
            math::Vector3d(m_vx * cos(yaw) + m_vy * cos(yaw + M_PI / 2.0), m_vx * sin(yaw) + m_vy * sin(yaw + M_PI / 2.0),
                            0));
            m_parent->SetAngularVel(math::Vector3d(0, 0, m_omega));
        }

GZ_REGISTER_MODEL_PLUGIN(RobotMovePlugin)
=======
        void cmdVelCB(const geometry_msgs::Twist::ConstPtr &cmd_msg) 
        {
            this->m_vx = cmd_msg->linear.x;
            this->m_vy = cmd_msg->linear.y;
            this->m_omega = cmd_msg->angular.z;
        }

        void CmdVelQueueThread()
        {
            static const double timeout = 0.01;

            while (this->m_gazebo->node()->ok())
            {
                this->m_cmd_vel_queue.callAvailable(ros::WallDuration(timeout));
            }
        }

        void UpdateChild()
        {
            float yaw = this->m_parent->WorldPose().Rot().Euler().Z();
            this->m_parent->SetLinearVel(
            math::Vector3d(this->m_vx * cos(yaw) + this->m_vy * cos(yaw + M_PI / 2.0), this->m_vx * sin(yaw) + this->m_vy * sin(yaw + M_PI / 2.0),
                            0));
            this->m_parent->SetAngularVel(math::Vector3d(0, 0, this->m_omega));
        }

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
>>>>>>> ff2589d8822fa20fecd780863cbd53ee42c486cf
