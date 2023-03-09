// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"uav_brocast");
    tf2_ros::StaticTransformBroadcaster static_broadcaster_map2word_ned;
    geometry_msgs::TransformStamped tfs_map2word_ned;
    tfs_map2word_ned.header.seq = 100;
    tfs_map2word_ned.header.stamp = ros::Time::now();
    tfs_map2word_ned.header.frame_id = "world_ned";
    tfs_map2word_ned.child_frame_id = "map";
    tfs_map2word_ned.transform.translation.x = 0;
    tfs_map2word_ned.transform.translation.y = 0;
    tfs_map2word_ned.transform.translation.z = 0;
    //----设置四元数:将 欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    tfs_map2word_ned.transform.rotation.x = qtn.getX();
    tfs_map2word_ned.transform.rotation.y = qtn.getY();
    tfs_map2word_ned.transform.rotation.z = qtn.getZ();
    tfs_map2word_ned.transform.rotation.w = qtn.getW();
    static_broadcaster_map2word_ned.sendTransform(tfs_map2word_ned);
    ros::spin();

    return 0;
}
