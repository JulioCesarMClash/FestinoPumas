#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <map>
#include <string>

#include <pose_estimation/PersonPose3D.h>
#include <pose_estimation/Keypoint3D.h>

class PointingDetector
{
    public:
        PointingDetector()
        {
            ros::NodeHandle nh;
            sub_ = nh.subscribe("/pose_3d", 10, &PointingDetector::poseCallback, this);
            pub_ = nh.advertise<std_msgs::String>("/pointing_direction", 10);
            std::cout << "Pointing detector node --- Soft by Joshua M" << std::endl;
        }

    private:
        ros::Subscriber sub_;
        ros::Publisher pub_;

        Eigen::Vector3d toVec(const pose_estimation::Keypoint3D& a, const pose_estimation::Keypoint3D& b)
        {
            return Eigen::Vector3d(b.x - a.x, b.y - a.y, b.z - a.z);
        }

        bool isPointing(const pose_estimation::Keypoint3D& shoulder, const pose_estimation::Keypoint3D& elbow, const pose_estimation::Keypoint3D& wrist)
        {
            Eigen::Vector3d v1 = toVec(shoulder, elbow);
            Eigen::Vector3d v2 = toVec(elbow, wrist);
            
            double v1_norm = v1.norm();
            double v2_norm = v2.norm();
            
            if (v1_norm < 1e-6 || v2_norm < 1e-6) return false;
            
            v1.normalize();
            v2.normalize();
            
            double cos_angle = v1.dot(v2);
            return cos_angle > 0.80;
        }

        void poseCallback(const pose_estimation::PersonPose3D::ConstPtr& msg)
        {
            std_msgs::String direction_msg;
            direction_msg.data = detectPointing(*msg);
            pub_.publish(direction_msg);
        }

        std::string detectPointing(const pose_estimation::PersonPose3D& person_pose)
        {
            std::map<std::string, pose_estimation::Keypoint3D> keypoints;
            for (const auto& kp : person_pose.keypoints)
            {
                keypoints[kp.name] = kp;
            }

            try
            {
                bool left = isPointing(keypoints.at("shoulder_left"), keypoints.at("elbow_left"), keypoints.at("wrist_left"));
                bool right = isPointing(keypoints.at("shoulder_right"), keypoints.at("elbow_right"), keypoints.at("wrist_right"));

                if (left && !right)
                {
                    ROS_INFO("Left");
                    return "left";
                }
                else if (right && !left)
                {
                    ROS_INFO("Right");
                    return "right";
                } else if (right && left)
                {
                    ROS_INFO("both");
                    return "both";
                }
            } catch (const std::out_of_range& e)
            {
                ROS_WARN("Wait for data: %s", e.what());
            }

            return "none";
        }
    };

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointing_detector_node");
    PointingDetector detector;
    ros::spin();
    return 0;
}