#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

double current_w;
double current_x;
double current_y;
double current_z;

// 读取imu的回调函数
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
}

// 主函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_imu_node");
	ros::NodeHandle nh;

    // 订阅IMU话题
	ros::Subscriber IMU_sub = nh.subscribe("/airsim_node/drone_1/imu/imu", 1, get_imu_callback);

    // 等待循环
	ros::spin();

	return 0;
}