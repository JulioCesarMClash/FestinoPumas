#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>
#include <move_base/move_baseAction.h> 

/*void move_baseF(double x, double y, double theta, double time_out, ros::Publisher& pubVel);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_action");
    ros::NodeHandle nh;

    ros::Rate rate(1000);
    ros::Publisher pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    move_baseF(0.1, 0.0, 0.0, 1.0, pubVel);
    return 0;
}

void move_baseF(double x, double y, double theta, double time_out, ros::Publisher& pubVel)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = x;
    vel_msg.linear.y = y;
    vel_msg.angular.z = theta;
    
    ros::Time init = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - init).toSec() < time_out)
    {
        pubVel.publish(vel_msg);
        ros::Duration(0.01).sleep();
    }
}*/



class MoveBaseAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_base::move_baseAction> as_;
    std::string action_name_;
    ros::Publisher pubVel;

public:
    MoveBaseAction(std::string name) :
        as_(nh_, name, boost::bind(&MoveBaseAction::executeCB, this, _1), false),
        action_name_(name)
        {
            as_.start();
            pubVel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        }
        ~MoveBaseAction(void){}

    void executeCB(const move_base::move_baseGoalConstPtr &goal)
    {
        ros::Rate rate(100000);
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = goal->x;
        vel_msg.linear.y = goal->y;
        vel_msg.angular.z = goal->theta;

        ros::Time init = ros::Time::now();
        while(ros::ok() && (ros::Time::now() - init).toSec() < goal->time_out)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                as_.setPreempted();
                return;
            }

            pubVel.publish(vel_msg);
            rate.sleep();
        }

        move_base::move_baseResult result;
        result.success = true;
        as_.setSucceeded(result);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_action_server");
    MoveBaseAction move_base("move_base");
    ros::spin();
    return 0;
}