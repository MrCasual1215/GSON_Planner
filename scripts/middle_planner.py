#!/usr/bin/python
from ourplanner.msg import global_path, pose
from nav_msgs.msg import OccupancyGrid, Path
from detection_msgs.msg import Groups,tracks
from scipy.interpolate import BSpline
import tf.transformations as tf
import tf2_ros
import numpy as np
import rospy
import math
from planner_utils.BIT_star import BITStar
from utils.group_manager import Group_manager
from utils.rviz_drawer import Rviz_drawer
from utils.map_manager import Map_manager
import traceback


class Middle_planner:
    def __init__(self) -> None:
        rospy.init_node("middle_planner_node")  

        self.global_path = None
        self.current_pose = pose()
        self.group_manager = Group_manager()
        self.rviz_drawer = Rviz_drawer()
        self.map_manager = Map_manager()
    
        rospy.Subscriber("/global_path", global_path,self.Global_path_cb,queue_size=1)
        rospy.Subscriber("/detection/group", Groups,self.group_manager.Group_add,queue_size=1)
        rospy.Subscriber("/tracker", tracks, self.group_manager.Group_update,queue_size=1)


        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_manager.Map_cb,queue_size=1)
        self.middle_path_pub = rospy.Publisher('/middle_path', global_path, queue_size=1)


        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.Middle_planning_radius = 5
        self.Middle_backward_dis = 2
        self.middle_path_flag = False


    def Global_path_cb(self,msg:global_path):
        '''
        获取global Reference
        '''
        self.global_path = global_path()
        self.global_path.length = msg.length
        self.global_path.goal_yaw = msg.goal_yaw
        self.global_path.path_x = msg.path_x
        self.global_path.path_y = msg.path_y
        
        self.last_path = msg



    def Get_robot_pose(self):
        '''
        获取机器人pose 6d
        '''
        pose = self.buffer.lookup_transform('map','base_link',rospy.Time(0))
        self.current_pose.x  = pose.transform.translation.x
        self.current_pose.y  = pose.transform.translation.y
        self.current_pose.z  = pose.transform.translation.z
        x = pose.transform.rotation.x
        y = pose.transform.rotation.y
        z = pose.transform.rotation.z
        w = pose.transform.rotation.w
        self.current_pose.roll, self.current_pose.pitch, self.current_pose.yaw = tf.euler_from_quaternion([x,y,z,w])



    def Socio_Spatial_Planning(self):
        """
        1) initialize
            - get robot's pose
            - update costmap
            - set global path 

        2) planning
            - planner init
            - need to replan?
                yes:self.group_manager.GPT_PERCEIVED
                    - set start and goal
                    - plan
                no: 
                    - return global path
            - return middle path
        """
        # for group in self.group_manager.groups:
        #     print(f"Group ID: {group.ids}, Number of people: {group.person_number}, Poses: {group.poses}")

        #-------------------------------------------initialize--------------------------------------------

        # get robot's pose
        self.Get_robot_pose()
        # update costmap
        self.map_manager.Get_middle_costmap(self.group_manager.ellipses_group, self.group_manager.polygons, self.group_manager.poses)

        # set global path 
        gloabl_path_length = self.global_path.length
        global_path_x = self.global_path.path_x
        global_path_y = self.global_path.path_y
        global_path_points = [[x,y] for x,y in zip(global_path_x, global_path_y)]
        self.map_manager.Set_global_path(global_path_points)

        #-------------------------------------------planning--------------------------------------------

        # planner init
        social_planner = BITStar(
            eta=1,
            iter_max=200, 
            costmap=self.map_manager.middle_costmap, 
            resolution=self.map_manager.resolution, 
            ellipses_group=self.group_manager.ellipses_group)

        end_collided_index  = self.global_path.length -1 


        # need to replan?
        MIDDLE_PLANNING = False

        dis = lambda x: math.sqrt((x[0] - self.current_pose.x)**2 + (x[1] - self.current_pose.y)**2) # distance^2
        min_dis_point = min(global_path_points,key=dis)             # 在global path 上找到距离robot's pose最近的point
        min_index = global_path_points.index(min_dis_point)         # 最近的索引
        min_distance = (dis(min_dis_point))                         # 最近的距离



        # 在 global path 上找到第一个碰撞的点
        for index in range(min_index, gloabl_path_length):          
            if self.map_manager._collision(index):                              # path[index] collided! 
                start_collided_index = index
                break      


        if 'start_collided_index' not in locals().keys():  
            if (min_distance > 1.0)  : # 虽然没有障碍物 但是距离global path较远，在global path上寻找taget
                MIDDLE_PLANNING = True
                for index in range(min_index, gloabl_path_length):
                    if dis(global_path_points[index]) > self.Middle_planning_radius:
                        end_collided_index = index
                        break
                if index == gloabl_path_length:
                    end_collided_index = gloabl_path_length -1
        else: 
            # global path 上有障碍物 
            # 寻找障碍物的边界

            for k in range(start_collided_index, gloabl_path_length):
                if self.map_manager._collision(k) == False:
                    end_collided_index = k
                    break
            if k == gloabl_path_length:
                end_collided_index = gloabl_path_length -1
        

            if (dis(global_path_points[start_collided_index]) < self.Middle_planning_radius): 
                # if robot's pose 与障碍物开始的距离小于middle planning半径，开启middle planning
                MIDDLE_PLANNING = True
                # 向后延长一段距离
                dis_sum = 0
                # rospy.loginfo("back ")

                for k in range(end_collided_index, gloabl_path_length):
                    dis = math.sqrt((global_path_x[k]-global_path_x[max(end_collided_index,k-1)])**2 + \
                                    (global_path_y[k]-global_path_y[max(end_collided_index,k-1)])**2)
                    dis_sum = dis_sum + dis
                    if dis_sum > self.Middle_backward_dis:
                        end_collided_index = k - 1 
                        break
                    if self.map_manager._collision(k): 
                        end_collided_index = k - 1
                        break
                if k == gloabl_path_length:
                    end_collided_index = gloabl_path_length -1


        if MIDDLE_PLANNING :
            # rospy.loginfo("e_index")
            self.e_index = end_collided_index 
            # set start and goal
            start = [self.current_pose.x - self.map_manager.origin_x, self.current_pose.y - self.map_manager.origin_y]
            goal = [global_path_x[self.e_index] - self.map_manager.origin_x ,global_path_y[self.e_index] - self.map_manager.origin_y]
            seclusion_path_x, seclusion_path_y = social_planner.planning(start, goal)
            if len(seclusion_path_x) == 1:
                rospy.loginfo("can't find middle path!")
                seclusion_path_x = []
                seclusion_path_y = []
                MIDDLE_PLANNING = False
            else:
                # Bspline 轨迹优化
                goal_index = min(self.e_index + 10, gloabl_path_length -1)
                seclusion_path_x.append(global_path_x[goal_index] - self.map_manager.origin_x)
                seclusion_path_y.append(global_path_y[goal_index] - self.map_manager.origin_y)
                seclusion_path = np.array([[x_,y_] for x_, y_ in zip(seclusion_path_x,seclusion_path_y)])

                k = min(2,len(seclusion_path)-1)  
                n = len(seclusion_path)
                t = np.concatenate(([0]*k, np.arange(n - k + 1), [n - k]*k))
                spl = BSpline(t, seclusion_path, k)
                x = np.linspace(0, n-k, 100)
                seclusion_path_opt = spl(x)

                seclusion_path_x, seclusion_path_y = [], []
                for point in seclusion_path_opt:
                    seclusion_path_x.append(point[0] + self.map_manager.origin_x)
                    seclusion_path_y.append(point[1] + self.map_manager.origin_y)
                
        else:
            seclusion_path_x = global_path_x
            seclusion_path_y = global_path_y


        # publish path
        path = global_path()
        path.length = len(seclusion_path_x)
        path.path_x = seclusion_path_x
        path.path_y = seclusion_path_y

        self.last_path = path 
        self.middle_path_flag = True
        self.middle_path_pub.publish(path)

        # rviz_show
        self.rviz_drawer.middle_path_rviz_show(path)



    def run(self):
        rate = rospy.Rate(5)  
        while not rospy.is_shutdown():
            try:
                if self.group_manager.GPT_PERCEIVED and self.global_path is not None:
                    self.Socio_Spatial_Planning()
                else:
                    rospy.loginfo("Middle planner is waiting........")
                    pass
            except Exception as e:
                if self.middle_path_flag:
                    self.middle_path_pub.publish(self.last_path)
                else:
                    path = global_path()
                    path.length = self.global_path.length
                    path.path_x = self.global_path.path_x
                    path.path_y = self.global_path.path_y
                    self.middle_path_pub.publish(path)
                rospy.logwarn(e)
            rate.sleep()



if __name__ == '__main__':
    middle_planner = Middle_planner()
    middle_planner.run()
