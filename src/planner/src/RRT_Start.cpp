// 导入标准库
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <random>
#include <vector>
#include <cmath>

// 导入消息文件
#include <nav_msgs/Path.h>

using namespace std;

// 定义RRT树结构体
struct Node
{
    int parent;
    double x, y;
    double cost;
};

// 定义RRT*规划器类
class RRTStarPlanner 
{
    private:
        // ROS节点句柄
        ros::NodeHandle nh_;
        // ROS发布者和订阅者
        ros::Subscriber map_sub_;
        ros::Publisher path_pub_;
        ros::Publisher marker_pub_;
        // RRT*参数
        double max_distance_;
        double goal_radius_;
        int max_iterations_;
        double map_origin_x_;
        double map_origin_y_;
        // RRT*数据结构
        vector<Node> nodes_;
        // 地图数据
        nav_msgs::OccupancyGrid map_;
        bool map_received_ = false;
        // 随机数生成器
        default_random_engine random_engine_;
        uniform_real_distribution<double> x_distribution_;
        uniform_real_distribution<double> y_distribution_;

        // 地图回调函数
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            map_ = *msg;
            map_origin_x_ = map_.info.origin.position.x;
            map_origin_y_ = map_.info.origin.position.y;
            map_received_ = true;
        }

        // 获取地图数据数组中单元格的索引
        int getIndex(double x, double y) {
            int i = (x - map_origin_x_) / map_.info.resolution;
            int j = (y - map_origin_y_) / map_.info.resolution;
            return i + j * map_.info.width;
        }

        // 获取树中距离给定点最近的节点
        int getNearestNodeIndex(const Node& node) {
            int nearest_index = 0;
            double min_distance = distance(node, nodes_[0]);
            for (int i = 1; i < nodes_.size(); i++) {
                double dist = distance(node, nodes_[i]);
                if (dist < min_distance) {
                    nearest_index = i;
                    min_distance = dist;
                }
            }
            return nearest_index;
        }

        // 计算两个结点之间的欧几里得距离
        double distance(const Node& node1, const Node& node2) {
            return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
        }

        // 计算结点与点之间的欧几里得距离
        double distance(const Node& node, const geometry_msgs::Point& point) {
            return sqrt(pow(node.x - point.x, 2) + pow(node.y - point.y, 2));
        }

        // 检查节点是否与地图发生冲突
        bool isInCollision(const Node& node) {
            if (map_.data[getIndex(node.x, node.y)] > 0) {
                return true;
            }
            return false;
        }

        // 对树重新布线以改善路径
        void rewire(Node& new_node) {
            for (int i = 0; i < nodes_.size(); i++) {
                Node& node = nodes_[i];
                if (distance(node, new_node) <= max_distance_ && node.cost + distance(node, new_node) < new_node.cost) {
                    if (!isInCollision(node)) {
                        new_node.parent = i;
                        new_node.cost = node.cost + distance(node, new_node);
                    }
                }
            }
        }
        
    public:
        // 这个是在实例化对象的时候就会执行的内容，类似于初始化
        RRTStarPlanner() : nh_("~"), random_engine_(random_device()()), x_distribution_(0, 1), y_distribution_(0, 1) 
        {
            // 初始化ROS发布者和订阅者
            map_sub_ = nh_.subscribe("map", 1, &RRTStarPlanner::mapCallback, this);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("markers", 1);
            // 初始化其他参数
            max_distance_ = 0.5;
            goal_radius_ = 0.2;
            max_iterations_ = 5000;     //最大迭代次数
            map_origin_x_ = 0.0;    //地图起点
            map_origin_y_ = 0.0;
        }

        // 运行RRT*算法
        void run() 
        {
            // 等待地图被接收
            while (ros::ok() && !map_received_) {
                ros::spinOnce();
            }
            // 生成RRT*
            nodes_.push_back(Node{ -1, 0, 0, 0.0 });
            for (int i = 0; i < max_iterations_; i++) {
                // 在自由空间中生成随机节点
                double x = x_distribution_(random_engine_) * map_.info.width * map_.info.resolution + map_origin_x_;
                double y = y_distribution_(random_engine_) * map_.info.height * map_.info.resolution + map_origin_y_;
                if (map_.data[getIndex(x, y)] > 0) {
                    continue;
                }
                // 在树中查找最近的节点
                int nearest_index = getNearestNodeIndex({ x, y });
                Node nearest_node = nodes_[nearest_index];
                // 计算到最近节点的开销
                double cost = nearest_node.cost + distance({ x, y }, nearest_node);
                if (cost > max_distance_) {
                    continue;
                }
                // 检查新节点是否发生冲突
                if (map_.data[getIndex(x, y)] > 0) {
                    continue;
                }
                // 将新节点连接到最近的节点
                Node new_node = { nearest_index, x, y, cost };
                // 对树重新布线以改善路径
                rewire(new_node);
                // 将新节点添加到树中
                nodes_.push_back(new_node);
                // 检查是否已达到目标
                if (distance({ 0, 0 }, { 10, 0 }) <= goal_radius_) {
                    ROS_INFO("Goal reached!");
                    // 生成从目标到起点的路径
                    nav_msgs::Path path;
                    int index = nodes_.size() - 1;
                    while (index != -1) {
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = nodes_[index].x;
                        pose.pose.position.y = nodes_[index].y;
                        pose.pose.position.z = 0.0;
                        path.poses.push_back(pose);
                        index = nodes_[index].parent;
                    }
                    reverse(path.poses.begin(), path.poses.end());
                    path.header.frame_id = "map";
                    path_pub_.publish(path);
                    return;
                }
            }
            ROS_WARN("Goal not reached after %d iterations.", max_iterations_);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_star_planner");
    RRTStarPlanner planner;
    planner.run();
    return 0;
}