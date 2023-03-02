#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "get_lidar_pose");
    ros::NodeHandle nh;

    // 创建 TF 订阅对象
    // 导入的"tf2_ros/transform_listener.h"
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(100);

    /*******************************主循环部分***********************************/
    while (ros::ok())
    {
        // 看try里面的内容就好了，这是调试的用法
        try
        {
            // 获取两个坐标系之间的关系
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("world_ned","drone_1/LidarSensor",ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系:父坐标系ID=%s",tfs.header.frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:子坐标系ID=%s",tfs.child_frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z
                    );
            ROS_INFO("Son1 相对于 Son2 的姿态关系:x=%.2f,y=%.2f,z=%.2f,w=%.2f",
                    tfs.transform.rotation.x,
                    tfs.transform.rotation.y,
                    tfs.transform.rotation.z,
                    tfs.transform.rotation.w
                    );
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
