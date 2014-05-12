#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "republish_pointcloud_service/RepublishPointcloud.h"

ros::Subscriber* sub;
ros::Publisher* pub;
ros::NodeHandle* n;
float subsampling;
std::string output;
std::string last_frame;
int last_seq;
bool first;
bool publish_empty;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_msg, *msg_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(msg_cloud);
    sor.setLeafSize(subsampling, subsampling, subsampling);
    sor.filter(*voxel_cloud);
    
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*voxel_cloud, output_msg);
    last_frame = input_msg->header.frame_id;
    last_seq = input_msg->header.seq;
	output_msg.header = input_msg->header;
    pub->publish(output_msg);
}

bool service_callback(republish_pointcloud_service::RepublishPointcloud::Request& req,
                      republish_pointcloud_service::RepublishPointcloud::Response& res)
{
    if (first && !req.republish) {
        return true;
    }
    if (!req.republish) {
        delete sub; sub = NULL;
        publish_empty = true;
        return true;
    }
    if (first) {
        pub = new ros::Publisher(n->advertise<sensor_msgs::PointCloud2>(req.output, 1));
        first = false;
    }
    sub = new ros::Subscriber(n->subscribe(req.input, 1, callback));
    subsampling = req.subsampling;    
    output = req.output;
    publish_empty = false;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "republish_pointcloud_service");
	n = new ros::NodeHandle();
	sub = NULL;
	pub = NULL;
	subsampling = 0.05;
	first = true;
	publish_empty = false;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    msg_cloud->points.resize(1);
    msg_cloud->points[0].x = 0.01;
    msg_cloud->points[0].y = 0.01;
    msg_cloud->points[0].z = 0.01;
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*msg_cloud, output_msg);
    
	ros::ServiceServer service = n->advertiseService("republish_pointcloud", &service_callback);
	ros::Rate rate(5);
	while (n->ok()) {
	    if (publish_empty && pub != NULL) {
	        output_msg.header.frame_id = last_frame;
            output_msg.header.stamp = ros::Time::now();
            output_msg.header.seq = last_seq + 1;
            ++last_seq;
	        pub->publish(output_msg);
	    }
	    rate.sleep();
	    ros::spinOnce();
	}

	return 0;
}
