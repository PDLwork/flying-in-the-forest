/*************************************************************
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *  
 * .............................................
 *          佛祖保佑             永无BUG
 ***********************************************************/
/**/

// 导入标准库
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/transforms.h>

// 导入消息文件
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;     //声明发布对象，这样可以在回调函数里面发布
pcl::PointCloud<pcl::PointXYZ>::Ptr pdl_map (new pcl::PointCloud<pcl::PointXYZ>);   //存储世界地图点云
sensor_msgs::PointCloud2 output;   //声明的输出的点云的格式

// 创建 TF 订阅对象
// 导入的"tf2_ros/transform_listener.h"
tf2_ros::Buffer buffer; 

// 获取雷达信息回调函数
void lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr & lidar_msg)
{
    try
        {
            // 获取两个坐标系之间的关系
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("world_ned","drone_1/LidarSensor",ros::Time(0), ros::Duration(0.5));

            // 需要添加头文件#include <pcl_conversions/pcl_conversions.h>
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_PointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*lidar_msg, *temp_PointCloud);

            //定义旋转矩阵
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z;   //平移向量

            Eigen::Quaterniond temp_qaternion(tfs.transform.rotation.w, tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z);     //四元数转旋转矩阵
            transform.rotate(temp_qaternion.matrix().cast<float>());

            // 执行变换，并将结果保存在新创建的 transformed_cloud 中
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*temp_PointCloud, *transformed_cloud, transform);

            // 合并点云
            *pdl_map = *pdl_map + *transformed_cloud;

            //转换成ros消息发布 
            pcl::toROSMsg(*pdl_map, output);
            output.header.frame_id = "world_ned";
            pub.publish (output);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("error:%s",e.what());
        }

}

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "mapping_1");
    ros::NodeHandle nh;
    
    // 
    ros::Subscriber person_info_sub = nh.subscribe("/airsim_node/drone_1/lidar/LidarSensor", 10, lidar_Callback);

    // 
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/output/Lidar", 10);

    tf2_ros::TransformListener listener(buffer);

    /*******************************主循环部分***********************************/
    ros::spin();

    return 0;
}