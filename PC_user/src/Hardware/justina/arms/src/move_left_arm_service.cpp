#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <arms/MoveArm.h>

class ArmMover
{
public:
    ArmMover() : move_group("left_arm")
    {
        move_group.setPlanningTime(10.0);
    }

    bool moveToPose(double x, double y, double z, double roll, double pitch, double yaw)
    {
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
        
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
        
    move_group.setPoseTarget(target_pose);
        
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        
    move_group.clearPoseTargets();
    return success;
    }


private:
    moveit::planning_interface::MoveGroupInterface move_group;
};

bool handle_move_request(arms::MoveArm::Request &req,
                         arms::MoveArm::Response &res,
                         ArmMover* mover)
{
    ROS_INFO("Solicitud de movimiento recibida: x=%.2f y=%.2f z=%.2f r=%.2f p=%.2f y=%.2f",
             req.x, req.y, req.z, req.roll, req.pitch, req.yaw);

    bool success = mover->moveToPose(req.x, req.y, req.z,
                                     req.roll, req.pitch, req.yaw);
    if (success)
    {
        res.success = true;
        res.message = "Movimiento exitoso.";
    }
    else
    {
        res.success = false;
        res.message = "Fallo al mover el brazo.";
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_left_arm_service_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ArmMover mover;
    ros::ServiceServer service = nh.advertiseService<arms::MoveArm::Request, arms::MoveArm::Response>( // Especifica los tipos
        "move_left_arm",
        boost::bind(&handle_move_request, _1, _2, &mover));

    ROS_INFO("[Main] Servicio listo");
    ros::waitForShutdown();
    return 0;
}