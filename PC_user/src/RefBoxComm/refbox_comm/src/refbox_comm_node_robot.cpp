#include <ros/ros.h>
#include <protobuf_comm/peer.h>

#include <refbox_protobuf_msgs/BeaconSignal.pb.h>
#include <refbox_protobuf_msgs/Pose2D.pb.h>
#include <refbox_protobuf_msgs/Team.pb.h>
#include <refbox_protobuf_msgs/Time.pb.h>

#include <refbox_protobuf_msgs/GameState.pb.h>

#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

//LOCALHOST
//#define HOST "localhost"
//ROBOCUP
//#define HOST "172.26.255.255"
//#define HOST "172.23.134.255"
//Labs
#define HOST "192.168.1.255"

//#define TEAM_COLOR "MAGENTA"
//#define TEAM_COLOR "CYAN"
#define TEAM_NAME "Pumas"
//#define CRYPTO_KEY "randomkey"
#define PUBLIC_PORT_S 4445
#define PUBLIC_PORT_R 4444

#include <iostream>
#include <typeinfo>

using namespace std;
using namespace protobuf_comm;

//--------------------------------ROBOT POSE
        float pose_x = 3.0f;
        float pose_y = 2.0f;
        float pose_ori = 45.0f;
        std::atomic<bool> pose_sem_th;
        std::atomic<bool> pose_sem_main;
//--------------------------------ROBOT POSE

class Handler {

    
    using BeaconSignal = llsf_msgs::BeaconSignal;
    using Pose2D = llsf_msgs::Pose2D;
    using Team = llsf_msgs::Team;
    using Time = llsf_msgs::Time;

    using GameState = llsf_msgs::GameState;

    private:
        std::string m_host;
        int m_port_s;
        int m_port_r;

        MessageRegister *m_mr;
        std::shared_ptr<ProtobufBroadcastPeer> m_public_peer;

        unsigned long int m_sequence_nr_;

        std::string m_team_name = TEAM_NAME;
        //Default values 1
        std::string m_robot_name = "Festino";
        int m_robot_number = 1;
        //Default values 2
        bool m_is_cyan = false;

        bool m_running;
        bool team_color_set;

        std::thread m_beacon_thread;

    public:
        Handler(std::string host, int port_s, int port_r)
            : m_host(host), m_port_s(port_s)
            , m_port_r(port_r)
            , m_mr(new MessageRegister()) {

            if(ros::param::has("~robot_name")){
                ros::param::get("~robot_name", m_robot_name);
            }
            if(ros::param::has("~robot_number")){
                ros::param::get("~robot_number", m_robot_number);
            }

            ROS_INFO_STREAM("Sending NAME: " << m_robot_name << "\t Sending NUMBER: " << m_robot_number);

            m_mr->add_message_type<BeaconSignal>();
            m_mr->add_message_type<GameState>();

            m_public_peer =  std::make_shared<ProtobufBroadcastPeer>(m_host, m_port_s, m_port_r, m_mr);

            
            m_public_peer->signal_received().connect(
                boost::bind(&Handler::handleRefboxMessage, this, _1, _2, _3, _4)
            );
            

            m_sequence_nr_ = 0;

            m_team_name = TEAM_NAME;

            m_running = true;
            team_color_set = false;
            

            m_beacon_thread = std::thread(&Handler::sendBeaconSignal, this);
            //m_beacon_thread.join();
            m_beacon_thread.detach();        
        }



        ~Handler() {
            m_running = false;
        }
        
/*
        void handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
        {
        }
*/

        void handleRefboxMessage(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
            std::shared_ptr<GameState> game_state;
            if ((game_state = std::dynamic_pointer_cast<GameState>(msg)))
            {
                //ROS_INFO_STREAM("----------------" << comp_id << " : " << msg_type);
                //ROS_INFO_STREAM(""<< game_state->ShortDebugString());
                
                if(!team_color_set){
                    auto cyan = game_state->team_cyan();
                    if (cyan == m_team_name){
                        m_is_cyan = true;
                        ROS_INFO_STREAM("COLOR SET CYAN ");
                    } else {
                        ROS_INFO_STREAM("COLOR SET MAGENTA ");
                    }
                    team_color_set = true;
                }
                
            }
        }

        void handleRecvErrorPrivate(boost::asio::ip::udp::endpoint &endpoint, std::string msg) {
            ROS_ERROR_STREAM("Error receiving on private port : " << msg);
        }

        void handleSendErrorPrivate(std::string msg) {
            ROS_ERROR_STREAM("Error sending on private port : " << msg);
        }

        // This method should notifies the refbox of a robot
        void sendBeaconSignal() {
            while (m_running) {
                if(!team_color_set){
                    continue;
                }

                auto cur_time = ros::Time::now();
                std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now =
                std::chrono::high_resolution_clock::now();
                std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                std::chrono::nanoseconds nanoseconds = now.time_since_epoch() - seconds;

                std::shared_ptr<BeaconSignal> msg(new BeaconSignal());

                Time *time = msg->mutable_time();
                time->set_sec(static_cast<google::protobuf::int64>(seconds.count()));
                time->set_nsec(static_cast<google::protobuf::int64>(nanoseconds.count()));

                Pose2D *pose = msg->mutable_pose();
                    
                /* Create Semaphore */
                while(!pose_sem_th.load()){
                    std::this_thread::yield();
                }
                pose->set_x(pose_x);
                pose->set_y(pose_y);
                pose->set_ori(pose_ori);
                pose_sem_main.store(true);
                pose_sem_th.store(false);
                /* Create Semaphore */

                Time *posetimestamp = pose->mutable_timestamp();
                posetimestamp->set_sec(time->sec());
                posetimestamp->set_nsec(time->nsec());

                msg->set_seq(++m_sequence_nr_);
                msg->set_number(m_robot_number);
                msg->set_team_name(m_team_name);
                msg->set_peer_name(m_robot_name);

                msg->set_team_color(m_is_cyan ? Team::CYAN : Team::MAGENTA);

                //ROS_INFO_STREAM("Sending: " << msg->ShortDebugString());

                m_public_peer->send(BeaconSignal::COMP_ID, BeaconSignal::MSG_TYPE, msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));//ROS_INFO_STREAM("Sending: ");
            }
        }
};

Handler* p;

/*//DEBUG
float debug_value_x = 0.0f;
float debug_value_y = 1.0f;
float debug_value_ori = 2.0f;
*/
int main(int argc, char** argv) 
{

    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    pose_sem_main.store(true);
    pose_sem_th.store(false);
        
    p = new Handler(HOST, PUBLIC_PORT_S, PUBLIC_PORT_R);

    ros::Rate r(10);
    while (ros::ok()) {

        //Obtaining robot location
	    geometry_msgs::PoseStamped tf_robot_pose;
	    tf::TransformListener listener_rob;
        tf::StampedTransform transform_rob;


        //real
        try {
            listener_rob.waitForTransform("/base_link","/map",   
                                   ros::Time(0), ros::Duration(1000.0));
            listener_rob.lookupTransform("/base_link","/map",   
                                   ros::Time(0), transform_rob);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        /* Create Semaphore */
        while(!pose_sem_main.load()){
            std::this_thread::yield();
        }
        /*//DEBUG
        debug_value_x += 0.01f;
        pose_x = debug_value_x;
        debug_value_y += 0.01f;
        pose_y = debug_value_y;
        debug_value_ori += 1.0f;
        pose_ori = debug_value_ori;
        */
        //real
        pose_x = transform_rob.getOrigin().x();
	    pose_y = transform_rob.getOrigin().y();
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_rob.getRotation());
        mat.getEulerYPR(yaw, pitch, roll);
        pose_ori = yaw;
        pose_sem_th.store(true);
        pose_sem_main.store(false);
        /* Create Semaphore */

        //std::cout << "Pose x: " << pose_x << " y: " << pose_y << " orientation: " << pose_ori << std::endl;

        ros::spinOnce();
	r.sleep();
    }
    return 0;
}
