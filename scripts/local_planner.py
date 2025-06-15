#!/usr/bin/python
from ourplanner.msg import  global_path, pose 
from geometry_msgs.msg import Twist, Point
from detection_msgs.msg import Groups,tracks
from detection_msgs.msg import tracks
import tf.transformations as tf
import numpy as np
import tf2_ros
import rospy
import math
from planner_utils.trajectory_generator import TrajectoryGenerator
from utils.rviz_drawer import Rviz_drawer
from planner_utils.optimizer import NmpcDbcfOptimizer, NmpcDcbfOptimizerParam



class Local_planner:
    def __init__(self) -> None:
        rospy.init_node("local_planner_node") 
        self.rviz_drawer = Rviz_drawer()
        self.middle_path = global_path()
        self.robot_pose = pose()
        self.goal_pose = pose()

        rospy.Subscriber("/middle_path", global_path,self.Middle_path_cb,queue_size=1)
        rospy.Subscriber("/tracker", tracks, self.Tracks_cb,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

        # parameters initialization 
        self.timestep = 0.1
        self.predicted_horizon = 20
        self.obstacles = np.array([[]])
        self.local_path_generator = TrajectoryGenerator(horizon=self.predicted_horizon)
        self.optimizer = NmpcDbcfOptimizer({},{},self.timestep, NmpcDcbfOptimizerParam())

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        

    def Get_robot_pose(self):
        """
        获取机器人位置 6d
        """
        pose = self.tf_buffer.lookup_transform('map','base_link',rospy.Time(0))
        self.robot_pose.x  = pose.transform.translation.x
        self.robot_pose.y  = pose.transform.translation.y
        self.robot_pose.z  = pose.transform.translation.z

        x = pose.transform.rotation.x
        y = pose.transform.rotation.y
        z = pose.transform.rotation.z
        w = pose.transform.rotation.w
        self.robot_pose.roll, self.robot_pose.pitch, self.robot_pose.yaw = tf.euler_from_quaternion([x,y,z,w])


    def Middle_path_cb(self,msg:global_path):
        '''
        获取 middle path
        '''
        self.middle_path.length = msg.length
        self.middle_path.path_x = msg.path_x
        self.middle_path.path_y = msg.path_y


    def Tracks_cb(self, msg:tracks):
        obsatcles = []
        for i, pose in enumerate(msg.track_pose_list.poses):
            obsatcle_states = []
            for step in range(self.predicted_horizon+1):
                obsatcle = np.array([pose.position.x + msg.track_vel_x_list[i] * step * self.timestep,\
                                     pose.position.y + msg.track_vel_y_list[i] * step * self.timestep])
                obsatcle_states.append(obsatcle)
            obsatcles.append(obsatcle_states)
        self.obstacles = np.array(obsatcles)

        

    def local_planning(self):
        ## robot pose
        self.Get_robot_pose()

        ## get local path
        pos = np.array([[self.robot_pose.x, self.robot_pose.y]])
        path = np.array([self.middle_path.path_x, self.middle_path.path_y]).T
        local_path = self.local_path_generator.generate_trajectory(pos, path)

        ## control
        state = np.array([[self.robot_pose.x, self.robot_pose.y ,self.robot_pose.yaw]]).T
        self.optimizer.setup(state, local_path, self.obstacles)
        u = self.optimizer.solve_nlp() 

        # print("Robot state:", state[0:2].T)
        # print("Local path:", local_path[5])
        # print("Control input:", u)  

        cmd_vel = Twist()
        cmd_vel.linear.x =  u[0]
        cmd_vel.linear.y =  0
        cmd_vel.angular.z = u[1]
        self.cmd_vel_pub.publish(cmd_vel)
        self.rviz_drawer.local_path_rviz_show(local_path)

        
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
                # rospy.loginfo("local planner is waiting......")
                pass
            rate.sleep()


if __name__ == '__main__':
    local_planner = Local_planner()
    local_planner.run()