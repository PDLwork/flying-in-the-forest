#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/conversions.h>

using namespace std;
using namespace octomap;

int z_up = 40;
int z_down = -14 * 2;

int main(int argc, char *argv[])
{
    /*****************************************初始化部分*********************************************/
    // 初始化ROS节点 
    ros::init(argc, argv, "generate_map"); 
    ros::NodeHandle nh;

    cout << endl;
    cout << "generating example map" << endl;

    OcTree tree(0.1);

    // 四周的墙
    for (int y=-10; y<90; y++)
    {
        int x = -10;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=-10; y<90; y++)
    {
        int x = 190;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int x=-10; x<190; x++)
    {
        int y = -10;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int x=-10; x<190; x++)
    {
        int y = 90;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }




    for (int y=20; y<40; y++)
    {
        int x = 20;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int x=-10; x<30; x++)
    {
        int y = 70;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=-10; y<30; y++)
    {
        int x = 90;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int x=50; x<90; x++)
    {
        int y = 30;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=55; y<90; y++)
    {
        int x = 90;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=20; y<90; y++)
    {
        int x = 120;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=-10; y<40; y++)
    {
        int x = 180;
        for (int z=z_down; z<z_up; z++)
        {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true);
        }
    }

    for (int y=27; y<33; y++)
    {
        for (int x=147; x<153; x++)
        {
            for (int z=z_down; z<z_up; z++)
            {
                point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
                tree.updateNode(endpoint, true);
            }
        }
    }

    cout << endl;
    tree.writeBinary("match_map.bt");

    return 0;
}