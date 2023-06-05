// 导入标准库
#include <ros/ros.h>

// 导入消息文件
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h> 
// #include <pcl/common/transforms.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace octomap;

// ros::Publisher world_pointcloud_publisher;     //声明发布对象，这样可以在回调函数里面发布
// pcl::PointCloud<pcl::PointXYZ>::Ptr world_map (new pcl::PointCloud<pcl::PointXYZ>);   //存储世界地图点云

// 用于存放起点与终点位置
geometry_msgs::Point current_start, current_end;

// 发布位置在rviz中显示
ros::Publisher marker_pub;

// 发布轨迹
nav_msgs::Path Flight_trajectory;
ros::Publisher path_pub;

// 存放地图
octomap::OcTree* octree;

// 定义三维坐标点结构体
struct Point3D 
{
    double x;
    double y;
    double z;

    // bool operator==(const Point3D& other) const {
    //     return x == other.x && y == other.y && z == other.z;
    // }
    bool operator==(const Point3D& other) const {
        // const double eps = 1e-8; // 设置一个非常小的误差范围
        const double eps = 0.499; // 设置一个非常小的误差范围   比单步跑距离要短一丢丢就行
        return std::abs(x - other.x) < eps
            && std::abs(y - other.y) < eps
            && std::abs(z - other.z) < eps;
    }
};

// 定义节点结构体
struct Node 
{
    Point3D pos;
    double g;   // 吊你妈的  有问题
    double h;
    double f;
    Node* parent;

    Node(Point3D p, double g, double h, Node* parent)
        : pos(p), g(g), h(h), f(g + h), parent(parent) {}

    bool operator<(const Node& other) const 
    {
        return h > other.h;  // 吊你妈的  有问题
    }
};


// 定义三维移动方向数组
vector<Point3D> moves = 
{
    { 0.5,    0,    0}, {-0.5,    0,    0},
    {   0,  0.5,    0}, {   0, -0.5,    0},
    {   0,    0,  0.5}, {   0,    0, -0.5},
    { 0.5,  0.5,    0}, {-0.5, -0.5,    0},
    { 0.5,    0,  0.5}, {-0.5,    0, -0.5}, 
    {   0,  0.5,  0.5}, {   0, -0.5, -0.5},
    { 0.5, -0.5,    0}, {-0.5,  0.5,    0},
    {   0,  0.5, -0.5}, {   0, -0.5,  0.5},
    { 0.5,    0, -0.5}, {-0.5,    0,  0.5},
    {-0.5,  0.5,  0.5}, { 0.5, -0.5, -0.5},
    {-0.5, -0.5,  0.5}, { 0.5,  0.5, -0.5},
    { 0.5,  0.5,  0.5}, {-0.5, -0.5, -0.5}
};
// vector<Point3D> moves = 
// {
//     { 1,    0,    0}, {-1,    0,    0},
//     {   0,  1,    0}, {   0, -1,    0},
//     {   0,    0,  1}, {   0,    0, -1},
// };


// 定义估价函数
double heuristic(Point3D a, Point3D b) 
{
    // return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}

// 判断一个点是否被占据即是否在障碍物中  后续可能需要改进为一个球内是否有障碍物
bool isInsideObstacle(Point3D p) 
{   
     // 查询一个立方体是否被占据
    for (double i = p.x-0.6; i <= p.x+0.6; i += 0.2)
    {
        for (double j = p.y-0.6; j <= p.y+0.6; j += 0.2)
        {
            for (double k = p.z-0.6; k <= p.z+0.6; k += 0.2)
            {
                octomap::point3d queryPt(i, j, k);
                octomap::OcTreeNode* result = octree->search(queryPt);

                if (result && octree->isNodeOccupied(result))  
                {
                    return true; 
                } 
            }
        }
    }
    return false; 
}

// 定义A*算法函数
vector<Point3D> aStar(Point3D start, Point3D goal) 
{
    // world_map->clear();
    // 将起点和终点4蛇5入保留一位小数  负数还有bug 以后再调 问题不大
    start.x = double(int(start.x *10 + 0.5)) * 0.1;
    start.y = double(int(start.y *10 + 0.5)) * 0.1;
    start.z = double(int(start.z *10 + 0.5)) * 0.1;
    // ROS_INFO("%f, %f ,%f",start.x,start.y,start.z);

    goal.x = double(int(goal.x *10 + 0.5)) * 0.1;
    goal.y = double(int(goal.y *10 + 0.5)) * 0.1;
    goal.z = double(int(goal.z *10 + 0.5)) * 0.1;
    // ROS_INFO("%f, %f ,%f",goal.x,goal.y,goal.z);

    // start.x = int(start.x *10 + 0.5);
    // start.y = int(start.y *10 + 0.5);
    // start.z = int(start.z *10 + 0.5);

    // goal.x = int(goal.x *10 + 0.5);
    // goal.y = int(goal.y *10 + 0.5);
    // goal.z = int(goal.z *10 + 0.5);

    // 初始化起点和终点节点
    Node* start_node = new Node(start, 0, heuristic(start, goal), nullptr);
    Node* goal_node = new Node(goal, 0, 0, nullptr);
    // ROS_INFO("%f, %f ,%f",start_node->pos.x,start_node->pos.y,start_node->pos.z);

    // 初始化open和closed集合
    priority_queue<Node> open_list;
    vector<Node*> closed_list;

    // 加入起点节点
    open_list.push(*start_node);

    // 搜索直到找到终点或open集合为空
    while (!open_list.empty()) 
    {
        // 取出open集合中f值最小的节点
        Node current_node = open_list.top();
        open_list.pop();

        // 将当前节点加入到closed集合中
        closed_list.push_back(new Node(current_node));
        // ROS_INFO("%f, %f ,%f",current_node.pos.x,current_node.pos.y,current_node.pos.z);
        // ROS_INFO("%f",current_node.g);

        // pcl::PointXYZ new_point;
        // new_point.x = current_node.pos.x;
        // new_point.y = current_node.pos.y;
        // new_point.z = current_node.pos.z;

        // world_map->points.push_back(new_point);
        // sensor_msgs::PointCloud2 world_map_output;   //定义消息类型
        // pcl::toROSMsg(*world_map, world_map_output);
        // world_map_output.header.frame_id = "map_ned"; 
        // world_pointcloud_publisher.publish (world_map_output);

        // 如果找到终点节点，则返回路径
        if (current_node.pos == goal_node->pos) 
        {
            vector<Point3D> path;
            Node* current = new Node(current_node);
            while (current != nullptr) 
            {
                geometry_msgs::PoseStamped current_path_point;
                current_path_point.pose.position.x = (current->pos.x);
                current_path_point.pose.position.y = (current->pos.y);
                current_path_point.pose.position.z = (current->pos.z);
                Flight_trajectory.poses.push_back(current_path_point);

                path.push_back(current->pos);
                current = current->parent;
            }
        reverse(path.begin(), path.end());  //将顺序倒转
        return path;
        }

        // 遍历当前节点的所有相邻节点
        for (auto move : moves) 
        {
            Point3D neighbor_pos = {current_node.pos.x + move.x, current_node.pos.y + move.y, current_node.pos.z + move.z};

            // 判断相邻节点是否合法（存在障碍物）
            if (isInsideObstacle(neighbor_pos)) 
            {
                continue;
            }

            // 判断相邻节点是否已经在closed集合中
            bool in_closed = false;
            for (auto node : closed_list) 
            {
                if (node->pos == neighbor_pos) 
                {
                    in_closed = true;
                    break;
                }
            }
            if (in_closed) 
            {
                continue;
            }

            // 计算相邻节点的g、h、f值
            double neighbor_g = current_node.g + heuristic(current_node.pos, neighbor_pos);
            double neighbor_h = heuristic(neighbor_pos, goal_node->pos);
            Node* neighbor_node = new Node(neighbor_pos, neighbor_g, neighbor_h, new Node(current_node));

            // 判断相邻节点是否已经在open集合中
            bool in_open = false;
            priority_queue<Node> open_list_tem = open_list;
            while (!open_list_tem.empty()) {
                Node node = open_list_tem.top();
                open_list_tem.pop();
                if (node.pos == neighbor_pos && node.f <= neighbor_node->f) 
                {
                    in_open = true;
                    break;
                }
            }
            if (in_open) 
            {
                continue;
            }

            // 将相邻节点加入open集合中
            open_list.push(*neighbor_node);
        }
    }

    // 如果open集合为空且还未找到终点，则无解
    return vector<Point3D>();
}

// 获取占据栅格地图的回调函数
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    // delete octree;
    // 将 OctomapBinary 消息解码为 OcTree 对象
    octomap::AbstractOcTree* abstractOcTree = octomap_msgs::msgToMap(*msg);
    octree = dynamic_cast<octomap::OcTree*>(abstractOcTree);
}

void getGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Point3D tempos = {msg->pose.position.y, msg->pose.position.x, -(msg->pose.position.z)};

    // 判断终点是否合理
    if (isInsideObstacle(tempos)) 
    {
        ROS_INFO("!!!The target point is too dangerous and needs to be reset!!!");
    }
    else
    {
        // 将上一终点设置为起点
        current_start.x = current_end.x;
        current_start.y = current_end.y;
        current_start.z = current_end.z;

        current_end.x = msg->pose.position.y;
        current_end.y = msg->pose.position.x;
        current_end.z = -(msg->pose.position.z);

        // 发布起点 红色
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map_ned";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE; // Marker类型为点
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5; // 点的大小
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0; // 颜色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.pose.position = current_start;
        marker_pub.publish(marker); // 发布Marker

        // 发布终点 绿色
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "map_ned";
        marker2.header.stamp = ros::Time::now();
        marker2.ns = "basic_shapes";
        marker2.id = 1;
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.scale.x = 0.5;
        marker2.scale.y = 0.5;
        marker2.scale.z = 0.5;
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;
        marker2.color.b = 0.0;
        marker2.color.a = 1.0;
        marker2.pose.orientation.x = 0.0;
        marker2.pose.orientation.y = 0.0;
        marker2.pose.orientation.z = 0.0;
        marker2.pose.orientation.w = 1.0;

        marker2.pose.position = current_end;
        marker_pub.publish(marker2); // 发布Marker

        // 将终点加入轨迹中
        Flight_trajectory.poses.clear();
        // geometry_msgs::PoseStamped current_position_end;
        // current_position_end.pose.position = current_end;
        // Flight_trajectory.poses.push_back(current_position_end);

        // 设置起点和终点
        Point3D start = {current_start.x, current_start.y, current_start.z};
        Point3D goal = {current_end.x, current_end.y, current_end.z};
        // Point3D start;
        // Point3D goal;
        // start.x = int(current_start.x *10 + 0.5);
        // start.y = int(current_start.y *10 + 0.5);
        // start.z = int(current_start.z *10 + 0.5);
        // goal.x = int(current_end.x *10 + 0.5);
        // goal.y = int(current_end.y *10 + 0.5);
        // goal.z = int(current_end.z *10 + 0.5);
        // ROS_INFO("%d, %d ,%d",start.x,start.y,start.z);
        // ROS_INFO("%d, %d ,%d",goal.x,goal.y,goal.z);

        // 输入起点终点，调用A*算法
        aStar(start, goal);
        
        // 发布轨迹结果
        Flight_trajectory.header.frame_id = "map_ned";
        Flight_trajectory.header.stamp = ros::Time::now();
        path_pub.publish(Flight_trajectory);
    }
}

int main(int argc, char **argv) 
{ 
    /*****************************************初始化部分*********************************************/
    // 初始化ROS节点 
    ros::init(argc, argv, "rrt_star_planner"); 
    ros::NodeHandle nh;

    current_end.x = 0;
    current_end.y = 0;
    current_end.z = -1;

    // 创建发布者
    path_pub = nh.advertise<nav_msgs::Path>("/rrt_path", 1);

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // world_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/world_map/pointcloud", 10);

    // 循环运行ROS节点
    ros::Rate loop_rate(10);

    // 订阅 OctomapBinary 消息
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, octomapCallback);

    // 订阅 goal 消息
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 1, getGoalCallback);

    /*****************************************主循环部分*********************************************/
    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}