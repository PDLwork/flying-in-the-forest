#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    /*******************************初始化部分***********************************/
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "mapping_1");
    ros::NodeHandle nh;
    
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);

    /*******************************主循环部分***********************************/
    return 0;
}
