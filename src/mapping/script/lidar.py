#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import numpy
import pcl
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField

def lidar_Callback(lidar_msg):
    # lidar_data = point_cloud2.read_points(lidar_msg)
    lidar_data = point_cloud2.read_points_list(lidar_msg)
    # print(type(lidar_data1))
    # for p in lidar_data:
    #     print(p)

    lidar_data = numpy.array(lidar_data)
    output.header.stamp = rospy.Time().now()
    output.height = 1
    output.width = lidar_msg.width
    output.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    output.is_bigendian = False
    output.point_step = 12
    # output.row_step = output.point_step * lidar_data.shape[0]
    output.is_dense = False
    output.data = numpy.asarray(lidar_data, numpy.float32).tostring()
    lidar_pub.publish(output)

if __name__ == '__main__':
    '''-------------------------------程序初始化部分------------------------------------'''
    # ROS节点初始化
    rospy.init_node('lidar', anonymous=True)

    rospy.Subscriber("/airsim_node/drone_1/lidar/LidarSensor", PointCloud2, lidar_Callback)

    lidar_pub = rospy.Publisher('/lidar_test', PointCloud2, queue_size=10)

    rate = rospy.Rate(10)

    output = PointCloud2()
    output.header.frame_id = "world_enu"; 
    '''-------------------------------程序循环部分------------------------------------'''
    # while not rospy.is_shutdown():
    #     output.header.stamp = rospy.Time().now()


    rospy.spin()