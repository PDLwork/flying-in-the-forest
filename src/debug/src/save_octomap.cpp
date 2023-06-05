#include <ros/ros.h>

#include <octomap_msgs/conversions.h>

bool flag = true;

// 获取占据栅格地图的回调函数
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    octomap::OcTree* octree;
    // 将 OctomapBinary 消息解码为 OcTree 对象
    octomap::AbstractOcTree* abstractOcTree = octomap_msgs::msgToMap(*msg);
    octree = dynamic_cast<octomap::OcTree*>(abstractOcTree);
    if (flag)
    {
        octree->writeBinary("map.bt");
        ROS_INFO("OK!!!");
        flag = false;
    }
}

int main(int argc, char *argv[])
{
    /*****************************************初始化部分*********************************************/
    // 初始化ROS节点 
    ros::init(argc, argv, "save_octomap"); 
    ros::NodeHandle nh;

    // 订阅 OctomapBinary 消息
    ros::Subscriber octomapSub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, octomapCallback);

    ros::spin();

    return 0;
}
