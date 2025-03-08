#! /usr/bin/python
from ourplanner.msg import global_path, pose
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import numpy as np
import tf2_ros
import rospy
import sys
sys.path.insert(0,'/home/sp/planner_ws/src/ourplanner/scripts')
from planner_utils.astar import AStarPlanner
from planner_utils.floyd_Bspline import Floyd_Bspline
from utils.rviz_drawer import Rviz_drawer
from utils.map_manager import Map_manager


class Global_planner:
    def __init__(self) -> None:
        rospy.init_node("global_planner_node") 

        self.goal = pose()
        self.current_pose = pose()
        self.rviz_drawer = Rviz_drawer()
        self.map_manager = Map_manager()
        # self.map_manager.Get_global_costmap()

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_manager.Map_cb,queue_size=10)
        self.global_path_pub = rospy.Publisher("/global_path",global_path,queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.New_Goal_cb, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


        # self.goal.x = 31
        # self.goal.y = -28

        # self.goal.x = -2
        # self.goal.y =  1

        # self.goal.x =  32.5
        # self.goal.y =  -43.6
    

        # self.goal.x =  -8.6
        # self.goal.y =  -3.0
    
        self.goal.x = 48.31
        self.goal.y = -7.73

        self.GLOBAL_PLANNING = True


        self.PREDEFINED = False
        self.predefined_rx = rospy.get_param('rx', [])
        self.predefined_ry = rospy.get_param('ry', [])

        rospy.loginfo("global_planner init success")
        
        
    def Get_robot_pose(self):
        """
        获取机器人位置 6d
        """
        pose = self.tf_buffer.lookup_transform('map','base_link',rospy.Time(0))
        self.current_pose.x  = pose.transform.translation.x
        self.current_pose.y  = pose.transform.translation.y
        self.current_pose.z  = pose.transform.translation.z

        x = pose.transform.rotation.x
        y = pose.transform.rotation.y
        z = pose.transform.rotation.z
        w = pose.transform.rotation.w
        self.current_pose.roll, self.current_pose.pitch, self.current_pose.yaw = tf.euler_from_quaternion([x,y,z,w])


    def New_Goal_cb(self,msg:PoseStamped):
        """
        rviz 中的 2d nav goal 来获取目标点 
        """
        rospy.loginfo("Got new goal !")
        self.goal.x = msg.pose.position.x
        self.goal.y = msg.pose.position.y
        self.goal.z = msg.pose.position.z
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.goal.roll,self.goal.pitch,self.goal.yaw = tf.euler_from_quaternion(quaternion)
        # self.GLOBAL_PLANNING = True

    def publish_global_path(self):
        path = global_path()
        path.length = self.path_length
        path.goal_yaw = self.goal.yaw
        path.path_x = self.path_x
        path.path_y = self.path_y
        self.global_path_pub.publish(path)
        self.rviz_drawer.global_path_rviz_show(path)




    def global_planning(self):
        if self.PREDEFINED:
            path = global_path()
            path.length = len(self.predefined_rx)
            path.path_x =  [self.predefined_rx[i]  for i in range(path.length)] 
            path.path_y =  [self.predefined_ry[i]  for i in range(path.length)] 
            self.global_path_pub.publish(path)
            self.rviz_drawer.global_path_rviz_show(path)
            self.GLOBAL_PLANNING = True
            self.map_sub.unregister()
            # rospy.loginfo("The global path was published successfully!")
            return




        # get global costmap
        self.map_manager.Get_global_costmap() 

        # get robot's pose
        self.Get_robot_pose()

        # astar planning
        start = [ self.current_pose.x - self.map_manager.origin_x,  self.current_pose.y - self.map_manager.origin_y]
        goal = [ (self.goal.x - self.map_manager.origin_x),  (self.goal.y - self.map_manager.origin_y)]
        astar = AStarPlanner(self.map_manager.global_costmap,0.05)
        rx,ry = astar.planning(start[0],start[1],goal[0],goal[1])
        rospy.loginfo("Astar planning finished!")
        # print(rx)
        # print(ry)


        # trajectory optimization
        floyd_bspline = Floyd_Bspline(self.map_manager.global_costmap,2,0.05)
        rx,ry = floyd_bspline.Trajectory_optimize(rx,ry,floyd=True, Bspline=True)
        
        self.path_x = [x + self.map_manager.origin_x for x in rx]
        self.path_y = [y + self.map_manager.origin_y for y in ry]


        # publish global path
        self.path_length = len(rx)
        if self.path_length == 0:
            rospy.logwarn("astar didn't find path!")
            return
        

        # print(self.path_x)
        # print(self.path_y)

        self.publish_global_path()
        self.GLOBAL_PLANNING = False
        self.map_sub.unregister()
        rospy.loginfo("The global path was published successfully!")


    def run(self):
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            try:
                if  self.GLOBAL_PLANNING:
                    self.global_planning()     
                else:
                    self.publish_global_path()
            except Exception as e:
                rospy.logwarn(e)
            rate.sleep()



if __name__ == '__main__':
    global_planner = Global_planner()
    global_planner.run()


