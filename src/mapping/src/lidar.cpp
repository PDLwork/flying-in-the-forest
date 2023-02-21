#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 

void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    ROS_INFO("lidar data width is:%d", lidar_msg -> width);
    ROS_INFO("lidar data size is:%ld", sizeof(lidar_msg -> data));
    ROS_INFO("lidar data size1 is:%ld", sizeof(lidar_msg -> data.size()));
}

int main (int argc, char **argv) 
{
    ros::init (argc, argv, "pcl_create"); 
	
	ros::NodeHandle nh; 
	// 创建发布者
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/fly", 1);
	// 实例化对象  用于存放PCL点云数据
	pcl::PointCloud<pcl::PointXYZ> cloud; 
	// 实例化对象  用于存放可以发布的点云形式，
	sensor_msgs::PointCloud2 output; 

    ros::Subscriber person_info_sub = nh.subscribe("/airsim_node/drone_1/lidar/LidarSensor", 10, lidar_Callback);

    // 等待循环
	ros::spin();

    return 0;
}