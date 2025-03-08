#! /usr/bin/python3


import os
import time
import rospy
from std_msgs.msg import Bool

def restart_planner():
    rospy.loginfo("Restarting local planner...")
    os.system("rosnode kill /local_planner_node")  # 先杀掉当前节点
    # time.sleep(2)  # 等待2秒，确保节点已经关闭
    os.system("rosrun ourplanner local_planner.py --stop_flag False")  # 重新启动节点

def llm_flag_callback(msg):
    if msg.data:  # 如果 /llm_flag 的值为 True，重启 local_planner
        restart_planner()
    else:
        rospy.loginfo("no need to reset")

def monitor_llm_flag():
    rospy.init_node('planner_monitor', anonymous=True)
    rospy.loginfo("local monitor node start")
    rospy.Subscriber("/llm_flag", Bool, llm_flag_callback)  # 订阅 /llm_flag 话题
    rospy.spin()  # 保持节点运行

if __name__ == "__main__":
    try:
        monitor_llm_flag()
    except rospy.ROSInterruptException:
        pass
