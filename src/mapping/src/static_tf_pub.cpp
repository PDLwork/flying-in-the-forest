/*
节点功能：
发布各个坐标系的TF树关系
Airsim的真他妈难用，还是自己写吧
*/

// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"static_tf_pub");
    ros::NodeHandle nh;

    /***********************************发布静态坐标***********************************************/
    tf2_ros::StaticTransformBroadcaster static_broadcaster_pub;

    // 世界坐标下由ned转换为enu
    geometry_msgs::TransformStamped tfs_map_ned2map_enu;
    tfs_map_ned2map_enu.header.stamp = ros::Time::now();
    tfs_map_ned2map_enu.header.frame_id = "map_ned";
    tfs_map_ned2map_enu.child_frame_id = "map_enu";
    tf2::Quaternion q_orig, q_rot, q_new;
    double roll = M_PI, pitch = 0, yaw = M_PI/2; // NED to ENU
    q_rot.setRPY(roll, pitch, yaw);
    q_orig.setRPY(0, 0, 0);
    q_new = q_rot * q_orig;
    tfs_map_ned2map_enu.transform.translation.x = 0.0;
    tfs_map_ned2map_enu.transform.translation.y = 0.0;
    tfs_map_ned2map_enu.transform.translation.z = 0.0;
    tfs_map_ned2map_enu.transform.rotation.x = q_new.x();
    tfs_map_ned2map_enu.transform.rotation.y = q_new.y();
    tfs_map_ned2map_enu.transform.rotation.z = q_new.z();
    tfs_map_ned2map_enu.transform.rotation.w = q_new.w();
    static_broadcaster_pub.sendTransform(tfs_map_ned2map_enu);

    // 八叉树map相对于world_ned的坐标关系
    geometry_msgs::TransformStamped tfs_map2map_ned;
    tfs_map2map_ned.header.stamp = ros::Time::now();
    tfs_map2map_ned.header.frame_id = "map_ned";
    tfs_map2map_ned.child_frame_id = "map";
    tfs_map2map_ned.transform.translation.x = 0;
    tfs_map2map_ned.transform.translation.y = 0;
    tfs_map2map_ned.transform.translation.z = 0;
    tfs_map2map_ned.transform.rotation.x = 0;
    tfs_map2map_ned.transform.rotation.y = 0;
    tfs_map2map_ned.transform.rotation.z = 0;
    tfs_map2map_ned.transform.rotation.w = 1;
    static_broadcaster_pub.sendTransform(tfs_map2map_ned);

    // lidar相对于drone_base_link的坐标关系
    geometry_msgs::TransformStamped tfs_lidar2drone;
    tfs_lidar2drone.header.stamp = ros::Time::now();
    tfs_lidar2drone.header.frame_id = "drone_base_link";
    tfs_lidar2drone.child_frame_id = "lidar";
    tfs_lidar2drone.transform.translation.x = 0;
    tfs_lidar2drone.transform.translation.y = 0;
    tfs_lidar2drone.transform.translation.z = -0.2;
    tfs_lidar2drone.transform.rotation.x = 0;
    tfs_lidar2drone.transform.rotation.y = 0;
    tfs_lidar2drone.transform.rotation.z = 0;
    tfs_lidar2drone.transform.rotation.w = 1;
    static_broadcaster_pub.sendTransform(tfs_lidar2drone);

    // 发布这个静态关系是因为无人机起飞的坐标并不是北东地的，会有偏航角，根据需要调试
    geometry_msgs::TransformStamped tfs_drone_start2map_ned;
    tfs_drone_start2map_ned.header.stamp = ros::Time::now();
    tfs_drone_start2map_ned.header.frame_id = "map_ned";
    tfs_drone_start2map_ned.child_frame_id = "drone_start";
    tf2::Quaternion q_orig1, q_rot1, q_new1;
    double roll1 = 0, pitch1 = 0, yaw1 = M_PI/2;
    q_rot1.setRPY(roll1, pitch1, yaw1);
    q_orig1.setRPY(0, 0, 0);
    q_new1 = q_rot1 * q_orig1;
    tfs_drone_start2map_ned.transform.translation.x = 0.0;
    tfs_drone_start2map_ned.transform.translation.y = 0.0;
    tfs_drone_start2map_ned.transform.translation.z = 0.0;
    tfs_drone_start2map_ned.transform.rotation.x = q_new1.x();
    tfs_drone_start2map_ned.transform.rotation.y = q_new1.y();
    tfs_drone_start2map_ned.transform.rotation.z = q_new1.z();
    tfs_drone_start2map_ned.transform.rotation.w = q_new1.w();
    static_broadcaster_pub.sendTransform(tfs_drone_start2map_ned);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
