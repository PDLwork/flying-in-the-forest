#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "gps.h"

// 全局变量
double init_longitude;
double init_latitude;
double init_altitude;
bool GPS_Initialization = false;

// 存放轨迹记录的对象
nav_msgs::Path Flight_trajectory;
ros::Publisher trajectory_pub;

// 存储的结构体
GpsDataType gps;
NedDataType ned;

// 读取gps的回调函数
void gps2ned_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	if (GPS_Initialization)
	{
		// 如果已经初始化则开始将gps转换成ned坐标
		ROS_INFO("current_longitude = %lf", gps_msg->longitude);
		ROS_INFO("current_latitude = %lf", gps_msg->latitude);
		ROS_INFO("current_altitude = %lf", gps_msg->altitude);

		// 实例化对象，每次回调都要？其实我不想这么做
		GpsTran gps_tran(init_longitude, init_latitude, init_altitude);

		// 提取
		gps.longitude = gps_msg->longitude;
		gps.latitude  = gps_msg->latitude;
		gps.altitude  = gps_msg->altitude;

		// 转换
		gps_tran.fromGpsToNed(ned, gps);

		// 打印查看
		ROS_INFO("NED of current GPS is:");
		ROS_INFO("x_north = %lf", ned.x_north);
		ROS_INFO("y_east = %lf", ned.y_east);
		ROS_INFO("z_down = %lf", ned.z_down);

		//发布轨迹
        Flight_trajectory.header.frame_id = "path";
        Flight_trajectory.header.stamp = ros::Time::now();  

        geometry_msgs::PoseStamped current_position;
        current_position.header = Flight_trajectory.header;

		// 取反一下  为了可以在rviz中更好的观察
        current_position.pose.position.x = ned.x_north;
        current_position.pose.position.y = -ned.y_east;
        current_position.pose.position.z = -ned.z_down;

        Flight_trajectory.poses.push_back(current_position);

        // ROS_INFO("( x:%0.6f, y:%0.6f, z:%0.6f)", ned.x_north, ned.y_east, ned.z_down);

		// 发布消息
        trajectory_pub.publish(Flight_trajectory);
	}
	else
	{
		init_longitude = gps_msg->longitude;
		init_latitude  = gps_msg->latitude;
		init_altitude  = gps_msg->altitude;

		GPS_Initialization = true;

		ROS_INFO("initial GPS is:");

		ROS_INFO("init_longitude = %lf", init_longitude);
		ROS_INFO("init_latitude = %lf", init_latitude);
		ROS_INFO("init_altitude = %lf", init_altitude);
	}
}

// 主函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpa_tran_node");
	ros::NodeHandle nh;

	// 订阅GPS话题
	ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/global_gps", 1, gps2ned_callback);

	// 发布轨迹话题
	trajectory_pub = nh.advertise<nav_msgs::Path>("Flight_trajectory", 10);

	// 等待循环
	ros::spin();

	return 0;
}
