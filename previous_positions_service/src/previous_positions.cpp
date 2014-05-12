#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "previous_positions_service/PreviousPosition.h"
#include <Eigen/Dense>

std::list<geometry_msgs::Pose> poses;

void callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    if (poses.empty()) {
        poses.push_front(*msg);
        return;
    }
    // just add if it is far enough from the last position
    Eigen::Vector3d p1(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector3d p2(poses.front().position.x, poses.front().position.y, poses.front().position.z);
    if ((p1 - p2).norm() < 0.1) {
        return;
    }
    poses.push_front(*msg);
    p2 = Eigen::Vector3d(poses.back().position.x, poses.back().position.y, poses.back().position.z);
    while (!poses.empty() && (p1 - p2).norm() > 2.0) {
        poses.pop_back();
        p2 = Eigen::Vector3d(poses.back().position.x, poses.back().position.y, poses.back().position.z);
    }
}

bool service_callback(previous_positions_service::PreviousPosition::Request& req,
                      previous_positions_service::PreviousPosition::Response& res)
{
    Eigen::Vector3d p1(poses.front().position.x, poses.front().position.y, poses.front().position.z);
    Eigen::Vector3d p2;
    for (std::list<geometry_msgs::Pose>::iterator i = poses.begin(); i != poses.end(); ++i) {
        p2 = Eigen::Vector3d(i->position.x, i->position.y, i->position.z);
        if ((p1 - p2).norm() > req.meters_back) {
            res.previous_pose.header.frame_id = "/map";
            res.previous_pose.pose = *i;
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "previous_positions_service");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("previous_position", &service_callback);
	ros::Subscriber sub = n.subscribe("/robot_pose", 1, &callback);
	ros::spin();
	
    return 0;
}
