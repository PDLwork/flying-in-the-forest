#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
用来订阅深度图与RGB图然后保存的,但是这样制作数据集好像有弊端
'''

import rospy
import numpy
import cv2

from sensor_msgs.msg import Image

#RGB图节点回调函数
def Front_RGB_Callback(msg):
    global img_rgb, GetRGBFlag
    buffer = numpy.frombuffer(msg.data, dtype=numpy.uint8)
    img_rgb = buffer.reshape(msg.height, msg.width, -1)     #此处得到的就是480*640*3的大小的数组
    GetRGBFlag = True

#深度图话题回调函数
def Front_Depth_Callback(msg):
    global img_depth, GetDepthFlag
    buffer = numpy.frombuffer(msg.data, dtype=numpy.float32)
    img_depth = buffer.reshape(msg.height, msg.width, -1)
    # img_depth = img_depth * 25.5
    GetDepthFlag = True

if __name__ == '__main__':
    '''-------------------------------程序初始化部分------------------------------------'''
    rospy.init_node('Data_subscriber', anonymous=True) # ROS节点初始化

    rospy.Subscriber("airsim_node/drone/front_center/Scene", Image, Front_RGB_Callback)   #创建话题订阅者，订阅前视彩图
    rospy.Subscriber("airsim_node/drone/front_center/DepthPlanar", Image, Front_Depth_Callback)   #创建话题订阅者，订阅前视深度图

    #创建本节点全局变量，订阅者会实时更新里面的值，供图像处理调用。
    img_rgb = numpy.zeros([480,640,3],dtype=numpy.uint8)
    GetRGBFlag = False
    img_depth = numpy.zeros([240,320],dtype=numpy.float32)
    GetDepthFlag = False

    image_index = 0

    rate = rospy.Rate(50)

    '''-------------------------------程序循环部分------------------------------------'''
    while not rospy.is_shutdown():
        if GetRGBFlag and GetDepthFlag:
            cv2.imwrite('/home/yun/Project/flying-in-the-forest/src/planner_leaning/MyDataset/RGB/{}_rgb.jpg'.format(image_index), img_rgb)
            cv2.imwrite('/home/yun/Project/flying-in-the-forest/src/planner_leaning/MyDataset/Depth/{}_depth.jpg'.format(image_index), img_depth)

            image_index = image_index + 1
            GetRGBFlag = False
            GetDepthFlag = False
            rospy.loginfo("图片保存成功!")
            rate.sleep()

    rospy.spin()