#! /usr/bin/python3

from ourplanner.msg import dynamic_obstacles, global_path, ellipses, pose, circle
from geometry_msgs.msg import Twist, Point, PoseStamped
from detection_msgs.msg import Groups,tracks
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
import time
import sys
sys.path.insert(0,'/home/sp/planner_ws/src/ourplanner/scripts')
from utils.rviz_drawer import Rviz_drawer
from utils.static_obstacle_manager import Static_obstacle_manager
from planner_utils.clf_cbf_nmpc import CLF_CBF_NMPC
import argparse

GroupEstimation = True

class Local_planner:
    def __init__(self,stop_flag=False) -> None:
        rospy.init_node("local_planner_node") 

        self.rviz_drawer = Rviz_drawer()
        self.middle_path = global_path()
        self.robot_pose = pose()
        self.goal_pose = pose()

        rospy.Subscriber("/global_path", global_path,self.Global_path_cb,queue_size=1)
        rospy.Subscriber("/middle_path", global_path,self.Middle_path_cb,queue_size=1)
        rospy.Subscriber("yolo/tracks",tracks, self.yolo_callback)
        rospy.Subscriber("/tracks", tracks, self.Tracks_cb,queue_size=1)
        rospy.Subscriber("/circles", circle, self.Circle_cb,queue_size=1)
        rospy.Subscriber("/llm_flag", Bool, self.LLM_cb, queue_size=1)
        rospy.Subscriber("/group", Groups, self.Group_cb, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

        # parameters initialization 
        self.other_agents_states = np.array([[]])
        self.circle = []
        self.ellipses = []

        self.predicted_horizon = 30
        self.time_step = 0.2


        self.static_obstacle_manager = Static_obstacle_manager(
            dbscan_eps= 0.5, 
            dbscan_min_sample=10, 
            window_size=20, 
            step_size=10,
            Rmax=0.3
        )

        self.clf_cbf_nmpc_solver = CLF_CBF_NMPC(
            n = self.predicted_horizon,      
            stp = self.time_step,    
            m_cbf = 5, 
            m_clf = 0,
            gamma = 0.5,
            alpha = 0.1
        )

        self.yolo_tracks = 0
        self.all_circles = []
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.Stop_flag = stop_flag
        print(f"Init Stop_flag:{stop_flag}")

    def LLM_cb(self, msg:Bool):
        if msg.data  and GroupEstimation :
            self.Stop_flag = msg.data


    def Global_path_cb(self,msg:global_path):
        '''
        获取global Reference
        '''

        self.global_path = [[x, y] for x,y in zip(msg.path_x, msg.path_y)]

    def yolo_callback(self, msg:tracks):
        self.yolo_tracks = len(msg.track_id_list)

    def Group_cb(self, msg:Groups):
        rospy.loginfo("local planner got group msg!!!!")
        if msg.group_list == []:
            pass
        else:
            rospy.Duration(2.0)
        self.Stop_flag = False


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



    def Get_robot_pose(self):
        [self.robot_pose.x, self.robot_pose.y, self.robot_pose.z ], \
        [self.robot_pose.roll, self.robot_pose.pitch, self.robot_pose.yaw], \
            = self.tf_trans('map','base_link')


    def Middle_path_cb(self,msg:global_path):
        '''
        获取 middle path
        '''
        self.middle_path.length = msg.length
        self.middle_path.path_x = msg.path_x
        self.middle_path.path_y = msg.path_y
    
    def Laser_cb(self, msg:LaserScan):

        trans, rot = self.tf_trans("map","laser_frame")
        # rospy.loginfo("x,y,yaw")
        # rospy.loginfo(trans[0])
        # rospy.loginfo(trans[1])
        # rospy.loginfo(rot[2])


        ranges = np.array(msg.ranges)
        points = []
        cnt = 0
        for i, r in enumerate(ranges):
            if r < 2.0 :
                angle = msg.angle_min + i * msg.angle_increment
                x_base = r * np.cos(angle)
                y_base = r * np.sin(angle)
                cnt = cnt + 1
                if cnt % 5 == 0:
                    # 使用tf变换将base_link坐标系下的点转换到map坐标系
                    x_map = trans[0] + x_base * np.cos(rot[2]) - y_base * np.sin(rot[2])
                    y_map = trans[1] + x_base * np.sin

        trans, rot = self.tf_trans("map","laser_frame")
        # rospy.loginfo("x,y,yaw")
        # rospy.loginfo(trans[0])
        # rospy.loginfo(trans[1])
        # rospy.loginfo(rot[2])


        ranges = np.array(msg.ranges)(rot[2]) + y_base * np.cos(rot[2])
        points.append((x_map, y_map))

        # rospy.loginfo(points)
        self.static_obstacle_manager.get_static_obstacle(np.array(points))
        # self.rviz_drawer.local_circle_show(self.static_obstacle_manager.all_circles)
        # rospy.loginfo(self.static_obstacle_manager.all_circles)


    def Circle_cb(self, msg:circle):
        pass
        # all_circles = []
        # for i in range(len(msg.x)):
        #     all_circles.append([(msg.x[i], msg.y[i]),msg.r[i]])
        # self.all_circles = all_circles

    def Tracks_cb(self, msg:tracks):
        pass

        # est_state = []
        # for i in range(len(msg.track_pose_list)):
        #     est_state.append([msg.track_pose_list[i][0], msg.track_pose_list[i][1]])

        # pred_state = []
        # for k in range(self.predicted_horizon):
        #     k_state = []
        #     for j in range(len(msg.track_pose_list)):  # agent j at timestep k
        #         agent_j_state_k = msg.track_pose_list[j] + \
        #         [msg.track_vel_x_list[j] * (k+1) * self.time_step, msg.track_vel_y_list[j] * (k+1) * self.time_step]
        #         k_state.append(agent_j_state_k)

        #     pred_state.append(k_state)

        # other_states = np.concatenate((np.array([est_state]),np.array(pred_state)),axis=0)
        # self.other_agents_states = other_states


    def Update_goal(self,r):
    
        rx = self.middle_path.path_x
        ry = self.middle_path.path_y
        x = self.robot_pose.x
        y = self.robot_pose.y
        final_goal_x = self.middle_path.path_x[self.middle_path.length-1]
        final_goal_y = self.middle_path.path_y[self.middle_path.length-1]


        # find the nearest waypoint 
        min_distance = 0
        min_index = 0
        for i in range(len(rx)):
            distance = (x-rx[i])**2 + (y-ry[i])**2
            if(distance < min_distance or min_index == 0):
                min_distance = distance
                min_index = i


        # near final goal     
        goal_dis = math.sqrt((x-final_goal_x)**2 + (y-final_goal_y)**2)
        if goal_dis <= r or min_index == self.middle_path.length-1:
            return np.array([[final_goal_x, final_goal_y, 0]]).T
        
        
        index = min_index
        dis_sum = min_distance

        goal_x = self.middle_path.path_x[min_index]
        goal_y = self.middle_path.path_y[min_index]

        next_x = self.middle_path.path_x[min(index+1,self.middle_path.length-1)]
        next_y = self.middle_path.path_y[min(index+1,self.middle_path.length-1)]


        while dis_sum < r  and index < self.middle_path.length:
            cur_x = self.middle_path.path_x[index]
            cur_y = self.middle_path.path_y[index]
            goal_x = self.middle_path.path_x[min(index+1,self.middle_path.length-1)]
            goal_y = self.middle_path.path_y[min(index+1,self.middle_path.length-1)]
            next_x = self.middle_path.path_x[min(index+2,self.middle_path.length-1)]
            next_y = self.middle_path.path_y[min(index+2,self.middle_path.length-1)]
            dis = math.sqrt((cur_x-goal_x)**2 + (cur_y-goal_y)**2)
            dis_sum += dis 
            index = index + 1 



        if next_x == goal_x:
            return np.array([[goal_x, goal_y, 0]]).T
        else:
            yaw = math.atan2((next_y - goal_y), (next_x - goal_x))
            return np.array([[goal_x, goal_y, yaw]]).T




    def local_planning(self):

        self.Get_robot_pose()
        x = np.array([[self.robot_pose.x, self.robot_pose.y ,self.robot_pose.yaw]]).T
        new_goal = self.Update_goal(1.5)
        # new_goal = np.array([[1.0, 1.0, 0.0]]).T


        delta_a = 2.0
        delta_b = 1.0 

        check_safe = lambda x, y, ellipse: ((((x - ellipse[0] )*math.cos(ellipse[4]) + (y - ellipse[1] )*math.sin(ellipse[4])) ** 2)/ ((ellipse[2]+delta_a)**2) + \
                            ((-(x - ellipse[0] )*math.sin(ellipse[4]) + (y - ellipse[1] )*math.cos(ellipse[4])) ** 2) / ((ellipse[3]+delta_b)**2)) - 1.0

        ellipses = []
        count = 0
        for ellipse in self.ellipses:
            if check_safe(self.robot_pose.x, self.robot_pose.y, ellipse) < 0:
                count = count + 1
                ellipses.append(ellipse)

        # rospy.loginfo(self.all_circles)

        global_states_sol, controls_sol, local_states_sol = self.clf_cbf_nmpc_solver.solve(
            init_st=x,
            goal_st=new_goal,
            others_states=self.other_agents_states,
            circles=self.all_circles,
            ellipses=ellipses
            )


        closest_point = min(self.global_path, key=lambda point: math.dist(point, [self.robot_pose.x, self.robot_pose.y]))
        closest_distance = math.dist(closest_point,  [self.robot_pose.x, self.robot_pose.y])
        # rospy.loginfo(f"closest_distance:{closest_distance}")
        v_lambda = 1.0
        if closest_distance < 0.4 and self.yolo_tracks <= 1:
            v_lambda = 1.5
            rospy.loginfo(f"v_lambda:{v_lambda}")   

        


        cmd_vel = Twist()
        u = controls_sol[0]
        if not self.Stop_flag:
            cmd_vel.linear.x =  u[0]*v_lambda
            cmd_vel.linear.y =  0
            cmd_vel.angular.z = u[1]*v_lambda
        else:
            rospy.loginfo("local planner STOPS!!!!!!!!!!!!!!!!!")
            cmd_vel.linear.x =  0
            cmd_vel.linear.y =  0
            cmd_vel.angular.z = 0    
        self.cmd_vel_pub.publish(cmd_vel)
        # rospy.loginfo("cmd _vel was published!")

        # rviz show
        self.rviz_drawer.local_path_rviz_show(global_states_sol)
        self.rviz_drawer.local_state_rviz_show([self.robot_pose.x, self.robot_pose.y], [new_goal.T[0][0], new_goal.T[0][1]])
        

        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.middle_path.length != 0:
                try:
                    self.local_planning()
                    pass
                except Exception as e:
                    rospy.logwarn(e)
            else:
                rospy.loginfo("local planner is waiting......")
                pass
            rate.sleep()


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == '__main__':
    # time.sleep(10)
    # 过滤掉 ROS 传递的特殊参数
    filtered_args = [arg for arg in sys.argv if not arg.startswith('__')]

    # 使用 argparse 解析传递的参数
    parser = argparse.ArgumentParser(description="Local Planner with Stop Flag Initialization")
    parser.add_argument('--stop_flag', type=str2bool, default=False, help="Initial Stop Flag state")
    args = parser.parse_args(filtered_args[1:])  # 过滤后的参数传递给 argparse
    print(args)

    # 初始化 Stop_flag
    stop_flag = args.stop_flag

    # 使用解析后的 stop_flag 初始化 Local_planner
    local_planner = Local_planner(stop_flag=stop_flag)
    local_planner.run()