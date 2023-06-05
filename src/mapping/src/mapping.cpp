                                    /***********************************************\
                                    *                                               *
                                    *                    _ooOoo_                    *
                                    *                   o8888888o                   *
                                    *                   88" . "88                   *
                                    *                   (| -_- |)                   *
                                    *                    O\ = /O                    *
                                    *                ____/`---'\____                *
                                    *              .   ' \\| |// `.                 *
                                    *               / \\||| : |||// \               *
                                    *             / _||||| -:- |||||- \             *
                                    *               | | \\\ - /// | |               *
                                    *             | \_| ''\---/'' | |               *
                                    *              \ .-\__ `-` ___/-. /             *
                                    *           ___`. .' /--.--\ `. . __            *
                                    *        ."" '< `.___\_<|>_/___.' >'"".         *
                                    *       | | : `- \`.;`\ _ /`;.`/ - ` : | |      *
                                    *         \ \ `-. \_ __\ /__ _/ .-` / /         *
                                    * ======`-.____`-.___\_____/___.-`____.-'====== *
                                    *                    `=---='                    *
                                    *                                               *
                                    * ............................................. *
                                    *          佛祖保佑             永无BUG           *
                                    \***********************************************/

/*
节点功能：
读取雷达点云以及雷达位姿转换成世界坐标下的点云
功能1：发布世界坐标下的点云，可以在rviz中显示（暂时关闭）
功能2：发布用于建占据栅格地图的点云
*/


// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_broadcaster.h>

// 导入消息文件
#include <sensor_msgs/PointCloud2.h>

ros::Publisher world_pointcloud_publisher;     //声明发布对象，这样可以在回调函数里面发布
pcl::PointCloud<pcl::PointXYZ>::Ptr world_map (new pcl::PointCloud<pcl::PointXYZ>);   //存储世界地图点云

ros::Publisher lidar_pointcloud_publisher;      //声明发布对象，这样可以在回调函数里面发布,发布雷达点云，用来建八叉树地图，原来的用不了

// 创建 TF 订阅对象
// 导入的"tf2_ros/transform_listener.h"
// tf2_ros::Buffer buffer; 

// 获取雷达信息回调函数
// 可以多加滤波，先不管先  目前可以用
void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr & lidar_msg)
{
    int takeoff_flag = ros::param::param("/take_off_flag",0);
    if (takeoff_flag)
    {
        // try用来调试是否读取到雷达的位姿，读取不到就跳过
        try
        {
            // 获取两个坐标系之间的关系，用来获取雷达的位姿
            // geometry_msgs::TransformStamped tfs = buffer.lookupTransform("map_ned","lidar",ros::Time(0), ros::Duration(0.1));


            // 开辟一个临时存储点云空间，存储当前雷达点云数据
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_PointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            // 将ros消息转换成pcl可以处理的形式   需要添加头文件#include <pcl_conversions/pcl_conversions.h>
            // 深度图转换到temp_PointCloud


            //
            pcl::fromROSMsg(*lidar_msg, *temp_PointCloud);


            // 统计滤波器 ，第一次滤除单独的点云噪声
            // 开辟一个临时存储点云空间，存储滤除噪声的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statistical_filter(new pcl::PointCloud<pcl::PointXYZ>);
            // 实例化滤波器 并设置参数
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical_filter;
            Statistical_filter.setMeanK(50); //K近邻搜索点个数
            Statistical_filter.setStddevMulThresh(1.0); //标准差倍数
            Statistical_filter.setNegative(false); //保留未滤波点（内点）
            // 输入点云滤波得到输出
            Statistical_filter.setInputCloud(temp_PointCloud);  //输入点云
            Statistical_filter.filter(*cloud_statistical_filter);  //保存滤波结果到cloud_statistical_filter

            // 发布雷达点云，用来建八叉树地图，原来的用不了 坐标系不一样
            sensor_msgs::PointCloud2 lidar_pointcloud_output;   //定义消息类型
            pcl::toROSMsg(*cloud_statistical_filter, lidar_pointcloud_output);
            lidar_pointcloud_output.header.frame_id = "lidar";     //设置在rviz中显示的坐标系
            lidar_pointcloud_output.header.stamp = ros::Time::now();
            lidar_pointcloud_publisher.publish(lidar_pointcloud_output);


            // //由雷达位姿定义旋转矩阵 将雷达采集点云转换为世界坐标下的点云
            // Eigen::Affine3f transform = Eigen::Affine3f::Identity();    //实例化旋转矩阵并设置为单位阵
            // transform.translation() << tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z;   //平移向量
            // Eigen::Quaterniond temp_qaternion(tfs.transform.rotation.w, tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z);     //四元数转旋转矩阵
            // transform.rotate(temp_qaternion.matrix().cast<float>());    //旋转向量
            // // 执行变换，并将结果保存在新创建的 transformed_cloud 中
            // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::transformPointCloud(*cloud_statistical_filter, *transformed_cloud, transform);


            // // 将点云合并至世界地图中
            // *world_map += *transformed_cloud;


            // // 体素网络滤波器，对点云进行降采样，防止产生过多点云
            // // 开辟一个临时存储点云空间，存储体素滤波后的点云
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelgrid_filter(new pcl::PointCloud<pcl::PointXYZ>);
            // // 实例化滤波器并设置参数
            // pcl::VoxelGrid<pcl::PointXYZ> Voxel_filter;
            // Voxel_filter.setLeafSize (0.05f, 0.05f, 0.05f);// 单位：m
            // // 输入点云并降采样，存储在cloud_voxelgrid_filter中,再转移到world_map中
            // Voxel_filter.setInputCloud (world_map);
            // Voxel_filter.filter (*cloud_voxelgrid_filter);
            // world_map.swap(cloud_voxelgrid_filter);


            // sensor_msgs::PointCloud2 world_map_output;   //定义消息类型
            // //将点云转换成可以发布的ros消息并发布
            // pcl::toROSMsg(*world_map, world_map_output);
            // world_map_output.header.frame_id = "map_ned";     //设置在rviz中显示的坐标系
            // world_pointcloud_publisher.publish (world_map_output);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("error:%s",e.what());
        }
    }
}

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;
    
    // 订阅来自雷达的消息
    ros::Subscriber person_info_sub = nh.subscribe("/airsim_node/drone1/lidar/LidarSensor", 10, lidar_Callback);

    // 用于发布世界雷达地图
    world_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/world_map/pointcloud", 10);
    lidar_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/Lidar/pointcloud", 10);

    // 订阅TF树
    // tf2_ros::TransformListener listener(buffer);

    /*******************************主循环部分***********************************/
    ros::spin();

    return 0;
}