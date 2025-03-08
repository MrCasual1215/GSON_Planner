#! /usr/bin/python3

from ourplanner.msg import dynamic_obstacles, global_path, ellipses, pose, circle
from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import tracks
import tf.transformations as tf
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import numpy as np
import tf2_ros
import rospy
import math
import sys
sys.path.insert(0,'/home/sp/planner_ws/src/ourplanner/scripts')
from utils.rviz_drawer import Rviz_drawer
from utils.static_obstacle_manager import Static_obstacle_manager


class Obstacle_manager:
    def __init__(self) -> None:
        rospy.init_node("obstacle_manager_node") 
        rospy.Subscriber("/scan", LaserScan, self.Laser_cb,queue_size=1)
        self.circles_pub = rospy.Publisher("/circles",circle,queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rviz_drawer = Rviz_drawer()
        self.ROI = 4.0
        self.static_obstacle_manager = Static_obstacle_manager(
            dbscan_eps= 0.5, 
            dbscan_min_sample=10, 
            window_size=20, 
            step_size=10,
            Rmax=0.5
        )

    def Laser_cb(self, msg:LaserScan):
        try:
            trans, rot = self.tf_trans("map","laser_frame")
        except Exception as e:
            return
        # rospy.loginfo("x,y,yaw")
        # rospy.loginfo(trans[0])
        # rospy.loginfo(trans[1])
        # rospy.loginfo(rot[2])


        ranges = np.array(msg.ranges)
        points = []
        cnt = 0
        for i, r in enumerate(ranges):
            if r < msg.range_max and r > msg.range_min  and r < self.ROI:
                angle = msg.angle_min + i * msg.angle_increment
                x_base = r * np.cos(angle)
                y_base = r * np.sin(angle)
                cnt = cnt + 1
                if cnt % 5 == 0:
                    # 使用tf变换将base_link坐标系下的点转换到map坐标系
                    x_map = trans[0] + x_base * np.cos(rot[2]) - y_base * np.sin(rot[2])
                    y_map = trans[1] + x_base * np.sin(rot[2]) + y_base * np.cos(rot[2])
                    points.append((x_map, y_map))
        # rospy.loginfo("points:")
        # rospy.loginfo(points)
        if points == []:
            return
        self.static_obstacle_manager.get_static_obstacle(np.array(points))
        self.rviz_drawer.local_circle_show(self.static_obstacle_manager.all_circles)
        # rospy.loginfo(self.static_obstacle_manager.all_circles)

        # publish 
        circle_msg = circle()
        x, y, r = [], [], []
        for circle_ in self.static_obstacle_manager.all_circles:
            x.append(circle_[0][0])
            y.append(circle_[0][1])
            r.append(circle_[1])
        circle_msg.cnt = len(self.static_obstacle_manager.all_circles)
        circle_msg.x = x
        circle_msg.y = y
        circle_msg.r = r
        self.circles_pub.publish(circle_msg)



    def tf_trans(self, target_frame, source_frame):
        pose = self.tf_buffer.lookup_transform(target_frame,source_frame,rospy.Time(0))
        tx  = pose.transform.translation.x
        ty  = pose.transform.translation.y
        tz  = pose.transform.translation.z
        transform = [tx, ty, tz]

        x = pose.transform.rotation.x
        y = pose.transform.rotation.y
        z = pose.transform.rotation.z
        w = pose.transform.rotation.w
        roll, pitch, yaw = tf.euler_from_quaternion([x,y,z,w])
        rotation = [roll, pitch, yaw]
        return transform, rotation


    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()

 

if __name__ == '__main__':
    obstacle_manager = Obstacle_manager()
    obstacle_manager.run()



