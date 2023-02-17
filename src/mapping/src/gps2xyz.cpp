#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "gps.h"

#include <sensor_msgs/Imu.h>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

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

double current_w;
double current_x;
double current_y;
double current_z;

double current_pose_x;
double current_pose_y;
double current_pose_z;


// 读取gps的回调函数
void gps2ned_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	if (GPS_Initialization)
	{
		// 如果已经初始化则开始将gps转换成ned坐标
		// ROS_INFO("current_longitude = %lf", gps_msg->longitude);
		// ROS_INFO("current_latitude = %lf", gps_msg->latitude);
		// ROS_INFO("current_altitude = %lf", gps_msg->altitude);

		// 实例化对象，每次回调都要？其实我不想这么做
		GpsTran gps_tran(init_longitude, init_latitude, init_altitude);

		// 提取
		gps.longitude = gps_msg->longitude;
		gps.latitude  = gps_msg->latitude;
		gps.altitude  = gps_msg->altitude;

		// 转换
		gps_tran.fromGpsToNed(ned, gps);

		// 打印查看
		// ROS_INFO("NED of current GPS is:");
		// ROS_INFO("x_north = %lf", ned.x_north);
		// ROS_INFO("y_east = %lf", ned.y_east);
		// ROS_INFO("z_down = %lf", ned.z_down);

		//发布轨迹
        Flight_trajectory.header.frame_id = "world_ned";
        Flight_trajectory.header.stamp = ros::Time::now();  

        geometry_msgs::PoseStamped current_position;
        current_position.header = Flight_trajectory.header;

		// 取反一下  为了可以在rviz中更好的观察
        current_position.pose.position.x = ned.x_north;
        current_position.pose.position.y = ned.y_east;
        current_position.pose.position.z = ned.z_down;

		current_pose_x = ned.x_north;
		current_pose_y = ned.y_east;
		current_pose_z = ned.z_down;

        Flight_trajectory.poses.push_back(current_position);

		// 发布消息
        trajectory_pub.publish(Flight_trajectory);
	}
	else
	{
		init_longitude = gps_msg->longitude;
		init_latitude  = gps_msg->latitude;
		init_altitude  = gps_msg->altitude;

		GPS_Initialization = true;

		// ROS_INFO("initial GPS is:");

		// ROS_INFO("init_longitude = %lf", init_longitude);
		// ROS_INFO("init_latitude = %lf", init_latitude);
		// ROS_INFO("init_altitude = %lf", init_altitude);
	}
}

// 读取imu的回调函数
// 暂时不使用  用官方的tf树更好用
void get_imu_callback(const sensor_msgs::Imu::ConstPtr& IMU_msg)
{
    current_w = IMU_msg->orientation.w;
    current_x = IMU_msg->orientation.x;
    current_y = IMU_msg->orientation.y;
    current_z = IMU_msg->orientation.z;

    // ROS_INFO("current_w = %lf", current_w);
    // ROS_INFO("current_x = %lf", current_x);
    // ROS_INFO("current_y = %lf", current_y);
    // ROS_INFO("current_z = %lf", current_z);



	//  5-1.创建 TF 广播器
    static tf2_ros::TransformBroadcaster broadcaster;
    //  5-2.创建 广播的数据(通过 pose 设置)
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    //  |----坐标系 ID
    tfs.child_frame_id = "drone1";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = current_pose_x;
    tfs.transform.translation.y = current_pose_y;
    tfs.transform.translation.z = current_pose_z;
    //  |--------- 四元数设置
    tfs.transform.rotation.x = current_x;
    tfs.transform.rotation.y = current_y;
    tfs.transform.rotation.z = current_z;
    tfs.transform.rotation.w = current_w;


    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);
}

// 主函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_tran_node");
	ros::NodeHandle nh;

	// 订阅GPS话题
	ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/global_gps", 1, gps2ned_callback);

	// 订阅IMU话题
	// 自己写的无人机姿态，暂时不用
	// ros::Subscriber IMU_sub = nh.subscribe("/airsim_node/drone_1/imu/imu", 1, get_imu_callback);

	// 发布轨迹话题
	trajectory_pub = nh.advertise<nav_msgs::Path>("Flight_trajectory", 10);

	// 等待循环
	ros::spin();

	return 0;
}
