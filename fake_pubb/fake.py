#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray
from detection_msgs.msg import Groups,tracks,Group


def create_pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose

def create_pose_array(x_list, y_list):
    pose_array = PoseArray()
    for x, y in zip(x_list, y_list):
        pose_array.poses.append(create_pose(x, y))
    return pose_array

def publish_loop():
    rospy.init_node('track_group_publisher', anonymous=True)

    track_pub = rospy.Publisher('/tracker', tracks, queue_size=10)
    group_pub = rospy.Publisher('/detection/group', Groups, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 构造 header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        # ========== TRACKS ==========
        track_msg = tracks()
        track_msg.header = header
        track_msg.track_id_list = [3, 4]
        track_msg.track_pose_list = create_pose_array([-2.2, 2.0], [1.8, 4.0])
        track_msg.track_vel_x_list = [0.5, 0.6]
        track_msg.track_vel_y_list = [0.1, -0.2]

        # ========== GROUPS ==========

        groups_msg = Groups()
        groups_msg.header = header


        group_msg = Group()
        group_msg.header = header
        group_msg.group_id_list = [1, 2]
        group_msg.group_pose_list = create_pose_array([1., 2], [-2, -2])
        group_msg.group_vel_x_list = [0.55, 0.2]
        group_msg.group_vel_y_list = [0.05, 0.3]
        groups_msg.group_list.append(group_msg)

        group_msg2 = Group()
        group_msg2.group_id_list = [5, 6]
        group_msg2.group_pose_list = create_pose_array([0, 1], [2, 3])
        group_msg2.group_vel_x_list = [0.1, 0.2]
        group_msg2.group_vel_y_list = [0.05, 0.3]
        groups_msg.group_list.append(group_msg2)



        # 发布消息
        track_pub.publish(track_msg)
        group_pub.publish(groups_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_loop()
    except rospy.ROSInterruptException:
        pass
