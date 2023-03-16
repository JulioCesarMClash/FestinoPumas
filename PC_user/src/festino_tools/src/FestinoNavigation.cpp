#include "festino_tools/FestinoNavigation.h"

bool FestinoNavigation::is_node_set;
actionlib_msgs::GoalStatus FestinoNavigation::_navigation_status;
actionlib_msgs::GoalStatus FestinoNavigation::_simple_move_status;
bool FestinoNavigation::_stop;

//Subscribers for stop signals
ros::Subscriber FestinoNavigation::subStop;
ros::Subscriber FestinoNavigation::subNavigationStop;
//Subscribers for checking goal-pose-reached signal
ros::Subscriber FestinoNavigation::subNavigationStatus;
ros::Subscriber FestinoNavigation::subSimpleMoveStatus;
//Publishers and subscribers for operating the simple_move node
ros::Publisher FestinoNavigation::pubSimpleMoveDist;
ros::Publisher FestinoNavigation::pubSimpleMoveDistAngle;
ros::Publisher FestinoNavigation::pubSimpleMoveLateral;
//Publishers and subscribers for mvn_pln
ros::Publisher FestinoNavigation::pubMvnPlnGetCloseXYA;
ros::Publisher FestinoNavigation::pubNavigationStop;
//Publishers and subscribers for localization
tf::TransformListener* FestinoNavigation::tf_listener;

bool FestinoNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "FestinoNavigation.->Setting ros node..." << std::endl;
    subStop                = nh->subscribe("/stop"                    , 10, &FestinoNavigation::callbackStop);
    subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &FestinoNavigation::callbackNavigationStop);
    subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &FestinoNavigation::callbackNavigationStatus);
    subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &FestinoNavigation::callbackSimpleMoveStatus);
    pubSimpleMoveDist      = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist", 10);
    pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
    pubSimpleMoveLateral   = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist_lateral", 10);
    pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
    pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
    tf_listener = new tf::TransformListener();

    is_node_set = true;
    _stop = false;
    _navigation_status.status  = actionlib_msgs::GoalStatus::PENDING;
    _simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    return true;
}

//Methods for checking if goal position is reached.
bool FestinoNavigation::isLocalGoalReached()
{
    return _simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::isGlobalGoalReached()
{
    return _navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::waitForLocalGoalReached(int timeOut_ms)
{
    FestinoNavigation::_stop = false;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    
    while(ros::ok() && FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    FestinoNavigation::_stop = false; //This flag is set True in the subscriber callback
    return FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
    FestinoNavigation::_stop = false;
    FestinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    while(ros::ok() && FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    FestinoNavigation::_stop = false;
    return FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

//Methods for robot localization
void FestinoNavigation::getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    FestinoNavigation::tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

void FestinoNavigation::getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    FestinoNavigation::tf_listener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

//These methods use the simple_move node
void FestinoNavigation::startMoveDist(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDist.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDistAngle.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startMoveLateral(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveLateral.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

bool FestinoNavigation::moveDist(float distance, int timeOut_ms)
{
    FestinoNavigation::startMoveDist(distance);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool FestinoNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    FestinoNavigation::startMoveDistAngle(distance, angle);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool FestinoNavigation::moveLateral(float distance, int timeOut_ms)
{
    FestinoNavigation::startMoveLateral(distance);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

//These methods use the mvn_pln node.
void FestinoNavigation::startGetClose(float x, float y, float angle)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = sin(angle/2);
    msg.pose.orientation.w = cos(angle/2);
    FestinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startGetClose(std::string location)
{
    
}

bool FestinoNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    FestinoNavigation::startGetClose(x,y,angle);
    return FestinoNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool FestinoNavigation::getClose(std::string location, int timeOut_ms)
{
    
}

void FestinoNavigation::stopNavigation()
{
    std_msgs::Empty msg;
    pubNavigationStop.publish(msg);
}


//Callbacks for subscribers
void FestinoNavigation::callbackStop(const std_msgs::Empty::ConstPtr& msg)
{
    FestinoNavigation::_stop = true;
}

void FestinoNavigation::callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg)
{
    FestinoNavigation::_stop = true;
}

void FestinoNavigation::callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    FestinoNavigation::_simple_move_status = *msg;
}

void FestinoNavigation::callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    FestinoNavigation::_navigation_status = *msg;
}
