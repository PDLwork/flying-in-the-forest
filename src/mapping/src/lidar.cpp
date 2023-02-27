#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>

ros::Publisher pcl_pub;
// 实例化对象  用于存放可以发布的点云形式，
// sensor_msgs::PointCloud2 output; 
// 实例化对象  用于存放PCL点云数据
// pcl::PointCloud<pcl::PointXYZ> cloud; 

void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr & lidar_msg)
{
    // ROS_INFO("lidar data width is:%d", lidar_msg -> width);
    // ROS_INFO("lidar data size is:%ld", sizeof(lidar_msg -> data));
    // ROS_INFO("lidar data size1 is:%ld", sizeof(lidar_msg -> data.size()));
	// pcl::PointCloud<pcl::PointXYZ> output;

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*lidar_msg, *output);
	
	// output.header.frame_id = "world_enu"; 
	// output->header.frame_id = "world_enu"; 
	// pcl_pub.publish(output);
}

int main (int argc, char **argv) 
{
    ros::init (argc, argv, "pcl_create"); 
	
	ros::NodeHandle nh; 
	// 创建发布者
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/fly", 1);

    ros::Subscriber person_info_sub = nh.subscribe("/airsim_node/drone_1/lidar/LidarSensor", 10, lidar_Callback);

    // 等待循环
	ros::spin();

    return 0;
}