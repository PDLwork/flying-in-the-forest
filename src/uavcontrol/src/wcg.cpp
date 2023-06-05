// 导入标准库
#include <ros/ros.h>

// 导入srv
#include "uavcontrol/Takeoff.h"
#include "uavcontrol/Land.h"

// 导入msg
#include "uavcontrol/VelCmd.h"
#include "uavcontrol/PoseCmd.h"

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    ros::init(argc, argv, "wcg"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh; // 创建node控制句柄

    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client = nh.serviceClient<uavcontrol::Takeoff>("/airsim_node/drone1/takeoff");

    // 调用服务前需要定义特定的调用参数
    uavcontrol::Takeoff takeoff;
    takeoff.request.waitOnLastTask = 1;

    //通过这两个publisher实现对无人机的速度控制和姿态控制
    ros::Publisher vel_publisher = nh.advertise<uavcontrol::VelCmd>("airsim_node/drone1/vel_cmd_body_frame", 1);

    ros::Rate rate(10);

    if(takeoff_client.call(takeoff)){
                ROS_INFO("Takeoff Succeed!");
                ros::param::set("/take_off_flag", 1);
            }

    /*******************************主循环部分***********************************/
    for(int i = 0; i < 120; i++)
    {
        uavcontrol::VelCmd velcmd;
        velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
        velcmd.twist.linear.x = 0; //x方向线速度(m/s)
        velcmd.twist.linear.y = -0.1;//y方向线速度(m/s)
        velcmd.twist.linear.z = -1.5;//z方向线速度(m/s)
        vel_publisher.publish(velcmd);
        rate.sleep();
    }

    for(int i = 0; i < 390; i++)
    {
        uavcontrol::VelCmd velcmd;
        velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
        velcmd.twist.linear.x = 4; //x方向线速度(m/s)
        velcmd.twist.linear.y = 0.0;//y方向线速度(m/s)
        velcmd.twist.linear.z = 0;//z方向线速度(m/s)
        vel_publisher.publish(velcmd);
        rate.sleep();
    }

    for(int i = 50; i > 0; i--)
    {
        uavcontrol::VelCmd velcmd;
        velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
        velcmd.twist.linear.x = 4; //x方向线速度(m/s)
        velcmd.twist.linear.y = -(i*0.15);//y方向线速度(m/s)
        velcmd.twist.linear.z = 0;//z方向线速度(m/s)
        vel_publisher.publish(velcmd);
        rate.sleep();
    }

    for(int i = 50; i > 0; i--)
    {
        uavcontrol::VelCmd velcmd;
        velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
        velcmd.twist.linear.x = 4; //x方向线速度(m/s)
        velcmd.twist.linear.y = 0;//y方向线速度(m/s)
        velcmd.twist.linear.z = 0;//z方向线速度(m/s)
        vel_publisher.publish(velcmd);
        rate.sleep();
    }

    /* code */
    return 0;
}
