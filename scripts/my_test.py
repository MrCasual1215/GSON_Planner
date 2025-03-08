#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import BSpline
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import numpy as np
import rospy
import tf

class PathOptimizer:
    def __init__(self):
        # 初始化节点
        rospy.init_node('path_optimizer', anonymous=True)

        # 订阅 /move_base_simple/goal 话题
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 发布到 /optimized_path 话题
        self.path_pub = rospy.Publisher("/optimized_path", Path, queue_size=10)

        # 存储接收到的goal
        self.goals = []
        self.timestamps = []
        self.current_time = 0.0


    def odom_callback(self, msg:Odometry):
        """订阅 /odom 话题，获取 header 时间戳并存储"""
        self.current_time = msg.header.stamp.to_sec()
        # rospy.loginfo(f"Got odom timestamp: {timestamp}")

    def goal_callback(self, msg:PoseStamped):
        # 获取目标点并存储
        self.goals.append([msg.pose.position.x, msg.pose.position.y])
        self.timestamps.append(self.current_time)

        # 当接收到足够的点时开始生成路径
        if len(self.goals) >= 2:
            self.linear_interpolation()
            self.export_points_to_txt()

    def linear_interpolation(self):
        # 时间序列
        time_series = self.timestamps
        # 位置序列
        position_series = self.goals

        # 定义插值步长
        step = 0.1

        # 插值结果存储
        new_time_series = []
        new_position_series = []

        for i in range(1, len(position_series)):
            # 当前点和下一个点
            x1, y1 = position_series[i-1]
            x2, y2 = position_series[i]
            t1 = time_series[i-1]
            t2 = time_series[i]
            
            # 计算距离
            distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # 计算需要的插值点数量
            num_points = int(np.floor(distance / step))
            
            # 插值
            for k in range(1, num_points + 1):
                t = k / (num_points + 1)
                x_new = x1 + t * (x2 - x1)
                y_new = y1 + t * (y2 - y1)
                time_new = t1 + t * (t2 - t1)
                
                new_position_series.append([x_new, y_new])
                new_time_series.append(time_new)

        # 添加最后一个原始点
        new_position_series.append(position_series[-1])
        new_time_series.append(time_series[-1])

        self.new_timestamps = new_time_series
        self.new_goals = new_position_series






    def export_points_to_txt(self, filename='tra.txt'):
        """
        将points导出为指定格式的txt文件
        格式：
        rx: [1.0, 2.0, ...]
        ry: [1.0, 2.0, ...]

        :param points: Nx2 数组，包含 (x, y) 坐标
        :param filename: 导出的txt文件名
        """
        with open(filename, 'w') as file:
            file.write("time = [")
            file.write(", ".join([str(t) for t in self.new_timestamps]))
            file.write("]\n")
            file.write("pose = [")
            file.write(", ".join([f"[{x}, {y}]" for x, y in self.new_goals]))
            file.write("]\n")
            rospy.loginfo(f"Data saved to {filename}")        





if __name__ == '__main__':
    try:
        path_optimizer = PathOptimizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
