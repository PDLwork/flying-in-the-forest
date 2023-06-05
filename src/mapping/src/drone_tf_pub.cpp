/*
获取GPS信息转换成世界坐标
获取IMU信息转换为字姿态
融合后发布在世界坐标系下的位姿
*/

// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "gps.h"

// 导入消息文件
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

// 存放是否受到消息标志位
bool received_GPS = false;
bool received_IMU = false;

//存放当前的位姿信息
double current_w;
double current_x;
double current_y;
double current_z;
double current_pose_x;
double current_pose_y;
double current_pose_z;

void Drone2worldPub()
{
	//  创建 TF 广播器
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
	tfs.header.frame_id = "drone_start";
	tfs.child_frame_id = "drone_base_link";
	tfs.header.stamp = ros::Time::now();
    //  坐标系相对信息设置
    tfs.transform.translation.x = current_pose_x;
    tfs.transform.translation.y = current_pose_y;
    tfs.transform.translation.z = current_pose_z;
    //  四元数设置
    tfs.transform.rotation.x = current_x;
    tfs.transform.rotation.y = current_y;
    tfs.transform.rotation.z = current_z;
    tfs.transform.rotation.w = current_w;

    //  广播器发布数据
    broadcaster.sendTransform(tfs);
}

// 读取GPS的回调函数
void gps2nedCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	// 在函数中定义变量，加快速度
	static GpsDataType gps;
	static NedDataType ned;
	static bool GPS_Initialization = false;
	static double init_longitude, init_latitude, init_altitude;

	if (GPS_Initialization)		// 如果已经初始化了就开始转换
	{
		// 实例化对象
		static GpsTran gps_tran(init_longitude, init_latitude, init_altitude);

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

		// 将转换完毕的位置存在全局变量中
		current_pose_x = ned.x_north;
		current_pose_y = ned.y_east;
		current_pose_z = ned.z_down;

		received_GPS = true;

		// 如果已经接收到两个消息，则调用发布位姿函数
		if (received_GPS && received_IMU) 
		{
			Drone2worldPub();

			// 重置变量以接收下一组消息
			received_GPS = false;
			received_IMU = false;
		}
	}
	else	// 没有初始化就先初始化
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
void getImuCallback(const sensor_msgs::Imu::ConstPtr& IMU_msg)
{
	current_w = IMU_msg->orientation.w;
    current_x = IMU_msg->orientation.x;
    current_y = IMU_msg->orientation.y;
    current_z = IMU_msg->orientation.z;

	received_IMU = true;

	// 如果已经接收到两个消息，则发布位姿
	if (received_GPS && received_IMU) 
	{
		Drone2worldPub();

		// 重置变量以接收下一组消息
		received_GPS = false;
		received_IMU = false;
	}

    // ROS_INFO("current_w = %lf", current_w);
    // ROS_INFO("current_x = %lf", current_x);
    // ROS_INFO("current_y = %lf", current_y);
    // ROS_INFO("current_z = %lf", current_z);
}

// 主函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_tf_pub");
	ros::NodeHandle nh;

	// 订阅GPS话题
	ros::Subscriber GPS_sub = nh.subscribe("/airsim_node/drone1/global_gps", 1, gps2nedCallback);

	// 订阅IMU话题
	ros::Subscriber IMU_sub = nh.subscribe("/airsim_node/drone1/imu/imu", 1, getImuCallback);

	// 等待循环
	ros::spin();

	return 0;
}
