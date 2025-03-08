import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import time
import math



class Static_obstacle_manager():
    def __init__(self, dbscan_eps, dbscan_min_sample, window_size, step_size, Rmax ) -> None:
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_sample = dbscan_min_sample
        self.window_size = window_size
        self.step_size = step_size
        self.Rmax = Rmax
        self.all_circles = []


    def dbscan_cluster(self):
            # 使用DBSCAN进行聚类
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_sample).fit(self.points)
        self.labels = clustering.labels_


    def dist(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def is_in_circle(self, p, c):
        return self.dist(p, c[0]) <= c[1]

    def get_circle_center(self, a, b, c):
        A = np.linalg.norm(np.array(b) - np.array(c))**2 * np.dot(np.array(a) - np.array(b), np.array(a) - np.array(c))
        B = np.linalg.norm(np.array(a) - np.array(c))**2 * np.dot(np.array(b) - np.array(a), np.array(b) - np.array(c))
        C = np.linalg.norm(np.array(a) - np.array(b))**2 * np.dot(np.array(c) - np.array(a), np.array(c) - np.array(b))
        return (A*np.array(a) + B*np.array(b) + C*np.array(c)) / (A + B + C + 1e-5)

    def circle_from_two_points(self, p1, p2):
        center = (np.array(p1) + np.array(p2)) / 2
        radius = self.dist(p1, p2) / 2
        return center, radius

    def circle_from_three_points(self, p1, p2, p3):
        center = self.get_circle_center(p1, p2, p3)
        radius = self.dist(center, p1)
        return center, radius

    def trivial_circle(self, points):
        if len(points) == 0:
            return np.array([0, 0]), 0
        elif len(points) == 1:
            return np.array(points[0]), 0
        elif len(points) == 2:
            return self.circle_from_two_points(points[0], points[1])
        else:
            return self.circle_from_three_points(points[0], points[1], points[2])

    def welzl(self, P, R):
        if len(P) == 0 or len(R) == 3:
            return self.trivial_circle(R)
        p = P[-1]
        d = self.welzl(P[:-1], R)
        if self.is_in_circle(p, d):
            return d
        return self.welzl(P[:-1], R + [p])

    def min_enclosing_circle(self, points):
        P = points.tolist()
        np.random.shuffle(P)
        return self.welzl(P, [])


    def sliding_window_fit(self, points):
        circles = []
        if len(points) - self.window_size < 0:
            return circles
        for start in range(0, len(points) - self.window_size + 1, self.step_size):
            end = start + self.window_size
            segment = points[start:end]
            if len(segment) >= 3:
                # t0 = time.time()
                center, radius = self.min_enclosing_circle(segment)
                radius = min(radius, self.Rmax)
                flag = True
                for point in segment:
                    if math.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2) > radius*2:
                        flag = False
                if flag:
                    circles.append((center, radius))
                # print(time.time()-t0)


        return circles
    
    def get_static_obstacle(self, points):
        if points == []:
            return
        self.points = points
        self.dbscan_cluster()
        all_circles = [] 
        for label in set(self.labels):
            if label == -1:
                continue  # 忽略噪声点
            cluster_points = points[self.labels == label]
            # t0 = time.time()
            circles = self.sliding_window_fit(cluster_points)
            # print(time.time()-t0)
            all_circles.extend(circles)
        self.all_circles = all_circles


    def visualize_point_cloud_and_circles(self):
        plt.figure(figsize=(8, 8))
        plt.scatter(self.points[:, 0], self.points[:, 1], s=5, label='Point Cloud')

        for center, radius in self.all_circles:
            circle = plt.Circle(center, radius, color='r', fill=False)
            plt.gca().add_patch(circle)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Point Cloud and Min Enclosing Circles')
        plt.axis('equal')
        plt.show()


def generate_fake_point_cloud():
    # 生成模拟点云数据
    angles1 = np.linspace(0, np.pi, 180)
    angles2 = np.linspace(np.pi, 2 * np.pi, 180)
    radius1 = 10  # 第一个圆的半径
    radius2 = 8   # 第二个圆的半径
    x_center1, y_center1 = 5, 5  # 第一个圆的中心
    x_center2, y_center2 = -10, -10  # 第二个圆的中心

    x1 = x_center1 + radius1 * np.cos(angles1)
    y1 = y_center1 + radius1 * np.sin(angles1)
    x2 = x_center2 + radius2 * np.cos(angles2)
    y2 = y_center2 + radius2 * np.sin(angles2)

    noise1 = np.random.normal(0, 0.5, x1.shape)
    noise2 = np.random.normal(0, 0.5, x2.shape)

    x1 += noise1
    y1 += noise1
    x2 += noise2
    y2 += noise2

    points1 = np.vstack((x1, y1)).T
    points2 = np.vstack((x2, y2)).T

    points = np.concatenate((points1, points2), axis=0)
    return points



if __name__ == "__main__":
    points = generate_fake_point_cloud()
    static_obstacle_manager = Static_obstacle_manager(
        dbscan_eps=2.0, 
        dbscan_min_sample=5, 
        window_size=10, 
        step_size=5,
        Rmax=1.0
        )
    static_obstacle_manager.get_static_obstacle(points)
    static_obstacle_manager.visualize_point_cloud_and_circles()

