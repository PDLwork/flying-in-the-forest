/*
节点功能：
读取无人机的位姿，记录位置，在世界坐标中发布已飞行轨迹
*/

// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// 导入消息文件
#include <nav_msgs/Path.h>

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    ros::init(argc, argv, "flight_path_record");
    ros::NodeHandle nh;

    // 创建 TF 订阅对象
    // 导入的"tf2_ros/transform_listener.h"
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    ros::Publisher Flight_trajectory_pub = nh.advertise<nav_msgs::Path>("/flight_path_record", 10);

    nav_msgs::Path Flight_trajectory;
    Flight_trajectory.header.frame_id = "world_ned";

    ros::Rate rate(5);

    /*******************************主循环部分***********************************/
    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("world_ned","drone_1/odom_local_ned",ros::Time(0), ros::Duration(0.5));

            geometry_msgs::PoseStamped current_position;
            current_position.pose.position.x = tfs.transform.translation.x;
            current_position.pose.position.y = tfs.transform.translation.y;
            current_position.pose.position.z = tfs.transform.translation.z;
            Flight_trajectory.poses.push_back(current_position);
            // 发布消息
            Flight_trajectory.header.stamp = ros::Time::now();
            Flight_trajectory_pub.publish(Flight_trajectory);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("error:%s",e.what());  
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}