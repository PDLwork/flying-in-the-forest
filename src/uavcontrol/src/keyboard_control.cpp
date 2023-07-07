/*
节点功能：
用键盘控制airsim中无人机飞行
*/

// 导入标准库
#include <ros/ros.h>

// 导入srv
#include "uavcontrol/Takeoff.h"
#include "uavcontrol/Land.h"

// 导入msg
#include "uavcontrol/VelCmd.h"
#include "uavcontrol/PoseCmd.h"

char kb=0;
bool exit_flag = 0;

int scanKeyboard()
{
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    if(select(1, &rfds, NULL, NULL, &tv)>0) 
        return getchar();
    else 
        return 0;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    ros::init(argc, argv, "keyboard_control"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle nh; // 创建node控制句柄

    system("stty raw -echo -F" "/dev/tty"); //用于启动键盘监听

    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client = nh.serviceClient<uavcontrol::Takeoff>("/airsim_node/drone/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<uavcontrol::Land>("/airsim_node/drone/land");

    // 调用服务前需要定义特定的调用参数
    uavcontrol::Takeoff takeoff;
    takeoff.request.waitOnLastTask = 1;
    uavcontrol::Land land;
    land.request.waitOnLastTask = 1;

    //通过这两个publisher实现对无人机的速度控制和姿态控制
    ros::Publisher vel_publisher = nh.advertise<uavcontrol::VelCmd>("airsim_node/drone/vel_cmd_body_frame", 1);
    ros::Publisher pose_publisher = nh.advertise<uavcontrol::PoseCmd>("airsim_node/drone/pose_cmd_body_frame", 1);

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    uavcontrol::VelCmd velcmd;
    velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0.5; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0.0;//y方向线速度(m/s)
    velcmd.twist.linear.z = -0.5;//z方向线速度(m/s)

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    uavcontrol::PoseCmd posecmd;
    posecmd.roll = 0; //x方向姿态(rad)
    posecmd.pitch = 0;//y方向姿态(rad)
    posecmd.yaw = 0;//z方向姿态(rad)
    posecmd.throttle = 0.596;//油门， （0.0-1.0）

    ros::Rate rate(50);

    /*******************************主循环部分***********************************/
    while(ros::ok())
    {
        //键盘控制无人机
        kb = scanKeyboard();
        switch (kb)
        {
        case '`':
            exit_flag = 1;
            break;
        case 't':    
        // 通过 .call(参数) 的函数对模拟器中的服务发起起飞请求，程序暂停直到该服务返回结果
            if(takeoff_client.call(takeoff)){
                ROS_INFO("Takeoff Succeed!");
                ros::param::set("/take_off_flag", 1);
            }else{
                ROS_ERROR("Failed to takeoff, exit!");\
                return 0;
            }
            break;
        case 'l':
        // 通过 .call(参数) 的函数对模拟器中的服务发起降落请求，程序暂停直到该服务返回结果
            if(land_client.call(land)){
                ROS_INFO("Land Succeed!");
            }else{
                ROS_ERROR("Failed to land, exit!");\
                return 0;
            }
            break;
        case 'w':
            velcmd.twist.linear.y =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.linear.x += 0.1;
            if(velcmd.twist.linear.x>5) velcmd.twist.linear.x=5;
            break;
        case 's':
            velcmd.twist.linear.y =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.linear.x -= 0.1;
            if(velcmd.twist.linear.x<-5) velcmd.twist.linear.x=-5;
            break;
        case 'd':
            velcmd.twist.linear.x =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.linear.y += 0.1;
            if(velcmd.twist.linear.y>5) velcmd.twist.linear.y=5;
            break;
        case 'a':
            velcmd.twist.linear.x =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.linear.y -= 0.1;
            if(velcmd.twist.linear.y<-5) velcmd.twist.linear.y=-5;
            break;
        case 'e':
            velcmd.twist.linear.x =0;
            velcmd.twist.linear.y =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.angular.z += 0.1;
            if(velcmd.twist.angular.z>1) velcmd.twist.angular.z=1;
            posecmd.roll =0;
            break;
        case 'q':
            velcmd.twist.linear.x =0;
            velcmd.twist.linear.y =0;
            velcmd.twist.linear.z =0;
            velcmd.twist.angular.z -= 0.1;
            if(velcmd.twist.angular.z<-1) velcmd.twist.angular.z=-1;
            posecmd.roll =0;
            break;
        case 'f':
            velcmd.twist.linear.x =0;
            velcmd.twist.linear.y =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z += 0.1;
            if(velcmd.twist.linear.z>2) velcmd.twist.linear.z=2;
            break;
        case 'r':
            velcmd.twist.linear.x =0;
            velcmd.twist.linear.y =0;
            velcmd.twist.angular.z =0;
            velcmd.twist.linear.z -= 0.1;
            if(velcmd.twist.linear.z<-2) velcmd.twist.linear.z=-2;
            break;
        default:
            velcmd.twist.linear.x -=sgn<double>(velcmd.twist.linear.x)*0.1;
            velcmd.twist.linear.y -=sgn<double>(velcmd.twist.linear.y)*0.1;
            velcmd.twist.linear.z -=sgn<double>(velcmd.twist.linear.z)*0.1;
            velcmd.twist.angular.z -=sgn<double>(velcmd.twist.angular.z)*0.03;
            break;
        }
        if(exit_flag == 1) break;
        vel_publisher.publish(velcmd);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
