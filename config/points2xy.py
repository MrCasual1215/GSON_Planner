#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import BSpline
from nav_msgs.msg import Path
import numpy as np
import rospy
import tf

class PathOptimizer:
    def __init__(self):
        # 初始化节点
        rospy.init_node('path_optimizer', anonymous=True)

        # 订阅 /move_base_simple/goal 话题
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # 发布到 /optimized_path 话题
        self.path_pub = rospy.Publisher("/optimized_path", Path, queue_size=10)

        # 存储接收到的goal
        self.goals = []
        self.frame_id = 'map'
        self.resolution =  2.0

    def goal_callback(self, msg):
        # 获取目标点并存储
        self.goals.append(msg.pose.position)

        # 当接收到足够的点时开始生成路径
        if len(self.goals) > 2:
            path = self.generate_path()
            self.export_points_to_txt(path)
            self.publish_path(path)

    def generate_path(self):
        # 从接收到的目标点中提取x和y坐标
        x = [goal.x for goal in self.goals]
        y = [goal.y for goal in self.goals]

        # 线性插值，按照每两个点间隔距离 r 插入点

        points = self.linear_interpolation(x, y, self.resolution)
        print(points)
        # points = [[goal.x, goal.y] for goal in self.goals ]
        # print((points))

        # 使用 B-Spline 优化曲线
        optimized_points = self.bspline_optimization(points)

        return optimized_points

    def linear_interpolation(self, x, y, resolution):
        # 线性插值函数，按距离 resolution 插入点
        points = []
        for i in range(len(x) - 1):
            dist = np.hypot(x[i+1] - x[i], y[i+1] - y[i])
            num_points = int(dist // resolution) -1  # 确定插入点的数
            points.append([x[i], y[i]])
            if num_points < 1:
                continue
            for j in range(1, num_points):
                t = j / float(num_points)
                interp_x = (1 - t) * x[i] + t * x[i+1]
                interp_y = (1 - t) * y[i] + t * y[i+1]
                points.append([interp_x, interp_y])
        points.append([x[-1], y[-1]])
        return points


    def bspline_optimization(self, points):
        ctrl_pts = points
        # 定义knot向量和次数
        k = 2 # B样条的次数
        n = len(ctrl_pts)
        t = np.linspace(0, 1, n - k + 1, endpoint=True)
        t = np.append(np.zeros(k), t)
        t = np.append(t, np.ones(k))

        # 创建B样条
        b_spline = BSpline(t, ctrl_pts, k)

        # 计算B样条曲线上的点
        x = np.linspace(0, 1, 100)
        spline_points = b_spline(x)
        return spline_points

    def publish_path(self, points):
        # 将优化后的路径发布到 /optimized_path
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()

        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path.poses.append(pose)

        rospy.loginfo("fxxking published!!!!")
        self.path_pub.publish(path)

    def get_orientation_from_points(self, points, current_point):
        # 计算路径点的方向
        idx = np.where((points == current_point).all(axis=1))[0][0]
        if idx < len(points) - 1:
            next_point = points[idx + 1]
        else:
            next_point = points[idx - 1]
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        yaw = np.arctan2(dy, dx)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return quat


    def export_points_to_txt(self, points, filename='gloabl_path.txt'):
        """
        将points导出为指定格式的txt文件
        格式：
        rx: [1.0, 2.0, ...]
        ry: [1.0, 2.0, ...]

        :param points: Nx2 数组，包含 (x, y) 坐标
        :param filename: 导出的txt文件名
        """
        rx = [point[0] for point in points]
        ry = [point[1] for point in points]

        with open(filename, 'w') as file:
            file.write("m_rx = [" + ", ".join(map(str, rx)) + "]\n")
            file.write("m_ry = [" + ", ".join(map(str, ry)) + "]\n")

        rospy.loginfo(f"Points exported to {filename}")

if __name__ == '__main__':
    try:
        path_optimizer = PathOptimizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
