#! /usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def goal_callback(msg):
    # 创建一个Marker消息
    marker_msg = Marker()
    marker_msg.header.frame_id = "map_ned"  # marker的坐标系
    marker_msg.header.stamp = rospy.Time.now()  # 时间戳

    # 设置marker的类型、位置、方向、颜色等参数
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = msg.pose.position.y
    marker_msg.pose.position.y = msg.pose.position.x
    marker_msg.pose.position.z = -msg.pose.position.z
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.5
    marker_msg.scale.y = 0.5
    marker_msg.scale.z = 0.5
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 1.0

    marker_pub.publish(marker_msg)

if __name__ == "__main__":
    rospy.init_node("get_goal")

    sub = rospy.Subscriber("/goal", PoseStamped, goal_callback, queue_size=1)

    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

    rospy.spin()