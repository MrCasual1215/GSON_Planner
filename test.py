import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN


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

def dist(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def is_in_circle(p, c):
    return dist(p, c[0]) <= c[1]

def get_circle_center(a, b, c):
    A = np.linalg.norm(np.array(b) - np.array(c))**2 * np.dot(np.array(a) - np.array(b), np.array(a) - np.array(c))
    B = np.linalg.norm(np.array(a) - np.array(c))**2 * np.dot(np.array(b) - np.array(a), np.array(b) - np.array(c))
    C = np.linalg.norm(np.array(a) - np.array(b))**2 * np.dot(np.array(c) - np.array(a), np.array(c) - np.array(b))
    return (A*np.array(a) + B*np.array(b) + C*np.array(c)) / (A + B + C)

def circle_from_two_points(p1, p2):
    center = (np.array(p1) + np.array(p2)) / 2
    radius = dist(p1, p2) / 2
    return center, radius

def circle_from_three_points(p1, p2, p3):
    center = get_circle_center(p1, p2, p3)
    radius = dist(center, p1)
    return center, radius

def trivial_circle(points):
    if len(points) == 0:
        return np.array([0, 0]), 0
    elif len(points) == 1:
        return np.array(points[0]), 0
    elif len(points) == 2:
        return circle_from_two_points(points[0], points[1])
    else:
        return circle_from_three_points(points[0], points[1], points[2])

def welzl(P, R):
    if len(P) == 0 or len(R) == 3:
        return trivial_circle(R)
    p = P[-1]
    d = welzl(P[:-1], R)
    if is_in_circle(p, d):
        return d
    return welzl(P[:-1], R + [p])

def min_enclosing_circle(points):
    P = points.tolist()
    np.random.shuffle(P)
    return welzl(P, [])


def sliding_window_fit(points, window_size=10, step_size=5):
    circles = []

    for start in range(0, len(points) - window_size + 1, step_size):
        end = start + window_size
        segment = points[start:end]
        if len(segment) >= 3:
            center, radius = min_enclosing_circle(segment)
            circles.append((center, radius))

    return circles

def visualize_point_cloud_and_circles(points, circles):
    plt.figure(figsize=(8, 8))
    plt.scatter(points[:, 0], points[:, 1], s=5, label='Point Cloud')

    for center, radius in circles:
        circle = plt.Circle(center, radius, color='r', fill=False)
        plt.gca().add_patch(circle)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Point Cloud and Min Enclosing Circles')
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":

    points = generate_fake_point_cloud()

    # 使用DBSCAN进行聚类
    clustering = DBSCAN(eps=2.0, min_samples=5).fit(points)
    labels = clustering.labels_

    all_circles = []

    # 对每个簇使用滑动窗口法进行最小包络圆形处理
    for label in set(labels):
        if label == -1:
            continue  # 忽略噪声点
        cluster_points = points[labels == label]
        circles = sliding_window_fit(cluster_points)
        all_circles.extend(circles)

    visualize_point_cloud_and_circles(points, all_circles)


