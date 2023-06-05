// 导入标准库
#include <ros/ros.h>
#include <random>

// 导入消息文件
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace octomap;

// 用于存放起点与终点位置
geometry_msgs::Point current_start, current_end;

// 发布位置在rviz中显示
ros::Publisher marker_pub;

// 发布轨迹
nav_msgs::Path Flight_trajectory;
ros::Publisher path_pub;

// 定义RRT常量
const double NODE_RADIUS = 2; // 优化的节点半径大小
const double MIN_DISTANCE = 0.5; // 最小距离
const int MAX_ITERATIONS = 5000; // 最大迭代次数
const double NODE_RADIUS1 = 4; // 优化的节点半径大小

// 定义RRT树结构体
struct RRTNode 
{
    geometry_msgs::Point position; // 节点坐标
    int parent; // 父节点
    double cost; // 从起点到该节点的代价
};

// 定义随机数生成器
std::random_device rd;      // 创建一个随机设备对象
std::mt19937 gen(rd());     // 创建一个 Mersenne Twister 引擎
std::uniform_real_distribution<double> dis(-20.0, 20.0);      // 调用dis(gen)就可以返回一个随机数
std::uniform_real_distribution<double> dis1(-1.0, 10.0); 

// 定义全局变量
std::vector<RRTNode> tree; // RRT树

// 存放地图
octomap::OcTree* octree;

// 计算两个点之间的距离
double distance(geometry_msgs::Point p1, geometry_msgs::Point p2) 
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// 找到最近的节点 返回最近的点在tree中的索引
int findNearestNode(geometry_msgs::Point p) 
{
    double min_distance = std::numeric_limits<double>::max();
    int nearest_node = 0;
    for (int i = 0; i < tree.size(); i++) 
    {
        double d = distance(p, tree[i].position);
        if (d < min_distance) 
        {
            min_distance = d;
            nearest_node = i;
        }
    }
    return nearest_node;
}

// 判断一个点是否被占据即是否在障碍物中  后续可能需要改进为一个球内是否有障碍物
bool isInsideObstacle(geometry_msgs::Point p) 
{
    // 查询一个点是否被占据
    octomap::point3d queryPt(p.x, p.y, p.z);
    octomap::OcTreeNode* result = octree->search(queryPt);

    // if (result) 
    if (result && octree->isNodeOccupied(result)) 
    // if (octree->isNodeOccupied(result)) 
    {
        // ROS_INFO("Point (%f, %f, %f) is occupied!", query.x(), query.y(), query.z());
        return true; 
    } 
    else 
    {
        // ROS_INFO("Point (%f, %f, %f) is not occupied.", query.x(), query.y(), query.z());
        return false; 
    }
}

// 判断两点之间的连线是否在障碍物中 没有就返回true
bool isPathObstructed(geometry_msgs::Point p1, geometry_msgs::Point p2) 
{
    // 获取起点和终点的坐标
    octomap::point3d start(p1.x, p1.y, p1.z);
    octomap::point3d end(p2.x, p2.y, p2.z);

    double resolution = octree->getResolution();
    double step_size = resolution / 2.0;
    point3d direction = end - start;
    direction.normalize();
    point3d current = start;
    while (current.distance(end) > step_size) {
        // Check if the current point is occupied
        OcTreeNode* node = octree->search(current);
        // if (octree->isNodeOccupied(node)) 
        // if (node != nullptr && octree->isNodeOccupied(node)) 
        if (node != nullptr && octree->isNodeOccupied(node)) 
        {
            // The current point is occupied
            return false;
        }
        // Move to the next point along the line segment
        current += direction * step_size;
    }
    // The entire line segment is free
    return true;

    // octomap::KeyRay ray;
    // if (octree->computeRayKeys(start, end, ray)) 
    // {
    //     for (auto key:ray) 
    //     {
    //         OcTreeNode* node = octree->search(key);
    //         if (octree->isNodeOccupied(node)) 
    //         {
    //             return true;
    //         }
    //     }
    // }
    // return false;
}

// 将一个新节点添加到树中
void addNewNode(geometry_msgs::Point input_position, int input_parent, double input_cost) 
{
    RRTNode node;
    node.position = input_position;
    node.parent = input_parent;
    node.cost = input_cost;
    tree.push_back(node);
}

// 查找邻近节点 在一个半径范围内 重新优化时候用 返回该点附近所有小于该半径的节点在tree中的索引
std::vector<int> findNearbyNodes(geometry_msgs::Point p, double radius) 
{ 
    std::vector<int> nearby_nodes; 
    for (int i = 0; i < tree.size(); i++) 
    { 
        double d = distance(p, tree[i].position); 
        if (d <= radius) 
        { 
            nearby_nodes.push_back(i); 
        } 
    } 
    return nearby_nodes; 
}

// 选择一个代价最小的节点 选择父节点
// 确认了该点位置之后，在一个半径内选择父节点而不是直接选择最近的 输入一个点位置和该点一个半径内所有可能父节点的索引  返回父节点在tree中的索引
// 如果连线都经过了障碍物，就返回-1
int chooseparent(geometry_msgs::Point p, std::vector<int> nearby_nodes) 
{ 
    double min_cost = std::numeric_limits<double>::max();  // 将一个double类型的变量初始化为最大值
    int temporary_parent = 0;  // 默认是起点
    for (int i = 0; i < nearby_nodes.size(); i++) 
    { 
        int node = nearby_nodes[i]; 
        if (isPathObstructed(tree[node].position, p))  // 判断两点之间的连线是否经过障碍物
        { 
            double cost = tree[node].cost + distance(tree[node].position, p); 
            if (cost < min_cost) 
            { 
                min_cost = cost; 
                temporary_parent = node; 
            } 
        } 
    } 
    if (min_cost < std::numeric_limits<double>::max())
    {
        return temporary_parent;
    }
    else
    {
        return -1;   // 所有连线都通过了障碍物 就返回-1 用于判断
    }
}

// 生成路径 输入一个tree中的索引 返回该索引到起点前一个点的一个向量，里面存放路径所有点的位置 即起点后一个位置到终点前的一个位置
std::vector<geometry_msgs::Point> generatePath(int node) 
{
    std::vector<geometry_msgs::Point> path; 

    geometry_msgs::PoseStamped current_position;

    while (node != 0) 
    {
        path.push_back(tree[node].position); 
        node = tree[node].parent; 

        current_position.pose.position = tree[node].position;
        Flight_trajectory.poses.push_back(current_position);
    } 

    path.push_back(tree[node].position); 
    // std::reverse(path.begin(), path.end());     // 将path中顺序调转
    return path; 
}

// RRT*算法  输入起点和终点位置
std::vector<geometry_msgs::Point> rrtStar(geometry_msgs::Point start, geometry_msgs::Point goal) 
{ 
    // 初始化RRT树 
    tree.clear(); 

    // 创建起点
    RRTNode root;
    root.position = start; 
    root.parent = 0; 
    root.cost = 0; 
    tree.push_back(root);

    // 迭代添加节点
    for (double i = 0; i < MAX_ITERATIONS; i++)
    {
        // ROS_INFO("%f", i);
        // 在一个长方体中随机生成一个点
        geometry_msgs::Point random_point;
        random_point.x = dis(gen);
        random_point.y = dis(gen);
        random_point.z = dis1(gen);

        // 找到最近的节点 返回在tree的点的索引
        int nearest_node = findNearestNode(random_point);
        geometry_msgs::Point nearest_point = tree[nearest_node].position;   // 获取该点的坐标

        // 生成一个新节点
        geometry_msgs::Point new_point;
        if (distance(random_point, nearest_point) < MIN_DISTANCE)
        {
            new_point = random_point;   //如果小于MIN_DISTANCE就直接选择该点
        } 
        else 
        {
            // 大于MIN_DISTANCE就选择连线上的一点
            // 这里先使用除法，后续优化代码在继续改进
            double proportion = MIN_DISTANCE / distance(random_point, nearest_point);
            new_point.x = nearest_point.x + proportion * (random_point.x - nearest_point.x);
            new_point.y = nearest_point.y + proportion * (random_point.y - nearest_point.y);
            new_point.z = nearest_point.z + proportion * (random_point.z - nearest_point.z);
        }

        // 判断该点是否在障碍物中 如果是有效点就继续执行操作
        if (isInsideObstacle(new_point) != true)
        {
            // 找该点一个范围内邻近的点
            std::vector<int> nearby_nodes = findNearbyNodes(new_point, NODE_RADIUS);

            // 选择一个代价最小的节点作为父节点
            int parent_indexes = chooseparent(new_point, nearby_nodes);
            if (parent_indexes > -1)  // 如果检测到了有效的父节点索引
            {
                // 添加新节点到树中
                double cost = tree[parent_indexes].cost + distance(tree[parent_indexes].position, new_point);
                // 上面三步分别实现了确定新的点位置、父节点索引、起点到该点的代价，然后调用函数将节点加入tree中
                addNewNode(new_point, parent_indexes, cost);
            
                std::vector<int> nearby_nodes1 = findNearbyNodes(new_point, NODE_RADIUS1);
                // 更新子节点的代价 重新选择路径
                for (int j = 0; j < nearby_nodes1.size(); j++) 
                {
                    int child = nearby_nodes1[j];
                    if (isPathObstructed(tree[child].position, new_point))       // 判断两点之间是否存在障碍物
                    {
                        double new_cost = cost + distance(new_point, tree[child].position);
                        if (new_cost < tree[child].cost) 
                        {
                            // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                            tree[child].parent = tree.size() - 1;   // 也就是最新加入点的索引 
                            tree[child].cost = new_cost;
                        }
                    }
                }
            }
        }
    }

    // 找到离终点最近的tree中的索引
    int final_node = findNearestNode(goal);

    // 生成路径
    std::vector<geometry_msgs::Point> path = generatePath(final_node);

    return path;
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
    geometry_msgs::PoseStamped current_position_end;
    current_position_end.pose.position = current_end;
    Flight_trajectory.poses.push_back(current_position_end);

    // 输入起点终点，调用RRT算法
    std::vector<geometry_msgs::Point> current_path = rrtStar(current_start, current_end);
    
    // 发布轨迹结果
    Flight_trajectory.header.frame_id = "map_ned";
    Flight_trajectory.header.stamp = ros::Time::now();
    path_pub.publish(Flight_trajectory);
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

    // tf2_ros::TransformListener listener(buffer);

    // 创建随机数生成器 为什么要用确定的种子呢？
    gen.seed(time(0));

    // 创建发布者
    path_pub = nh.advertise<nav_msgs::Path>("/rrt_path", 1);

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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