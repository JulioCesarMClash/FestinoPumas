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

            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<geometry_msgs::Twist>("cmd_vel", 1,
                                                          boost::bind(&RobotMovePlugin::cmdVelCB, this, _1),
                                                          ros::VoidPtr(), &m_cmd_vel_queue);

            this->m_cmd_vel_cb_thread = 
                boost::thread(boost::bind(&RobotMovePlugin::CmdVelQueueThread, this));
            this->m_update =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&RobotMovePlugin::UpdateChild, this));

        }

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