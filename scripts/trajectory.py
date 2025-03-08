#! /usr/bin/python3
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
import rospy
import math
import os

# 全局队列，存储 odom 的时间戳和 goal 的 x, y 坐标
odom_queue = deque()
goal_queue = deque()

# 用于存储插值后的时间戳和坐标
timestamps = []
poses = []

# 插值结果发布到 /interpolated_positions
marker_pub = None

def odom_callback(msg):
    """订阅 /odom 话题，获取 header 时间戳并存储"""
    timestamp = msg.header.stamp.to_sec()
    odom_queue.append(timestamp)
    # rospy.loginfo(f"Got odom timestamp: {timestamp}")

def goal_callback(msg):
    """订阅 /move_base_simple/goal 话题，获取 x, y 坐标并存储"""
    x = msg.pose.position.x
    y = msg.pose.position.y
    goal_queue.append((x, y))
    # rospy.loginfo(f"Got goal position: x={x}, y={y}")

def distance(x1, y1, x2, y2):
    """计算两点之间的距离"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def interpolate_positions():
    """基于距离的线性插值，并发布插值后的点"""
    if len(goal_queue) > 1 and len(odom_queue) > 1:
        # 获取两个相邻的 goal 坐标和对应的时间戳
        (x1, y1) = goal_queue.popleft()
        (x2, y2) = goal_queue[0]
        t1 = odom_queue.popleft()
        t2 = odom_queue[0]

        # 计算两点之间的距离
        dist = distance(x1, y1, x2, y2)

        if dist > 0:
            num_points = int(dist / 0.1)  # 按每 0.1 米插值
            x_vals = np.linspace(x1, x2, num=num_points)
            y_vals = np.linspace(y1, y2, num=num_points)
            time_vals = np.linspace(t1, t2, num=num_points)

            # 创建 MarkerArray 用于显示插值点
            marker_array = MarkerArray()

            for i, (x_interp, y_interp, t_interp) in enumerate(zip(x_vals, y_vals, time_vals)):
                # 记录时间和坐标
                timestamps.append(t_interp)
                poses.append([x_interp, y_interp])

                # 创建 marker 来可视化点
                marker = Marker()
                marker.header.frame_id = "map"  # 或者使用 "odom" 作为 frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "interpolated_points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x_interp
                marker.pose.position.y = y_interp
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05  # 点的大小
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

            # 发布 marker array
            marker_pub.publish(marker_array)
            rospy.loginfo(f"Published {len(marker_array.markers)} interpolated markers")

def save_data_to_file():
    """将插值后的时间戳和坐标保存到 .txt 文件"""
    filename = os.path.expanduser("~/interpolated_positions.txt")
    with open(filename, 'w') as file:
        # 写入时间戳列表
        file.write("time = [")
        file.write(", ".join([str(t) for t in timestamps]))
        file.write("]\n")

        # 写入坐标列表
        file.write("pose = [")
        file.write(", ".join([f"[{x}, {y}]" for x, y in poses]))
        file.write("]\n")

    rospy.loginfo(f"Data saved to {filename}")

def main():
    global marker_pub
    rospy.init_node('interpolator_node')

    # 订阅 /odom 和 /movebase_simple/goal 话题
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    # 创建一个发布器，发布到 /interpolated_positions 用于显示
    marker_pub = rospy.Publisher('/interpolated_positions', MarkerArray, queue_size=10)

    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        interpolate_positions()
        rate.sleep()

    rospy.loginfo(f"time:{timestamps}")
    rospy.loginfo(f"poses:{poses}")


    # 在节点关闭时保存数据
    save_data_to_file()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
