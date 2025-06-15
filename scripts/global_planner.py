#!/usr/bin/python
from ourplanner.msg import global_path, pose
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import tf2_ros
import rospy
from planner_utils.astar import AStarPlanner
from planner_utils.floyd_Bspline import Floyd_Bspline
from utils.rviz_drawer import Rviz_drawer
from utils.map_manager import Map_manager


class Global_planner:
    def __init__(self):
        rospy.init_node("global_planner_node") 

        self.load_yaml()
        self.planning_finished = False
        self.current_pose = pose()
        self.rviz_drawer = Rviz_drawer()
        self.map_manager = Map_manager()
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_manager.Map_cb,queue_size=10)
        self.global_path_pub = rospy.Publisher("/global_path",global_path,queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.New_Goal_cb, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("global_planner init success")


    def load_yaml(self):
        self.goal_predefine = rospy.get_param('global_planner/goal_predefined', False)
        if self.goal_predefine:
            self.goal = pose()
            self.goal.x =  rospy.get_param('global_planner/goal_x', 0.0)
            self.goal.y =  rospy.get_param('global_planner/goal_y', 0.0)
        else:
            self.goal = None

        self.path_predefined = rospy.get_param('global_planner/path_predefined', False)
        if self.path_predefined:
            self.predefined_rx = rospy.get_param('global_planner/path_x', [])
            self.predefined_ry = rospy.get_param('global_planner/path_y', [])
        self.param = rospy.get_param('global_planner/planner_params') 

        
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
        self.planning_finished = False
        rospy.loginfo("Got new goal !")
        self.goal = pose()
        self.goal.x = msg.pose.position.x
        self.goal.y = msg.pose.position.y
        self.goal.z = msg.pose.position.z
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.goal.roll,self.goal.pitch,self.goal.yaw = tf.euler_from_quaternion(quaternion)


    def publish_global_path(self):
        path = global_path()
        path.length = self.path_length
        path.goal_yaw = self.goal.yaw
        path.path_x = self.path_x
        path.path_y = self.path_y
        self.global_path_pub.publish(path)
        self.rviz_drawer.global_path_rviz_show(path)



    def global_planning(self):
        if self.path_predefined:
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
        
        if self.goal is None:
            rospy.logwarn("Waiting for the goal...!")
            return
        
        if self.planning_finished:
            self.publish_global_path()
            return


        # get robot's pose
        self.Get_robot_pose()

        # astar planning
        start = [ self.current_pose.x - self.map_manager.origin_x,  self.current_pose.y - self.map_manager.origin_y]
        goal = [ (self.goal.x - self.map_manager.origin_x),  (self.goal.y - self.map_manager.origin_y)]
        astar = AStarPlanner(self.map_manager.global_costmap,0.05)
        rx,ry = astar.planning(start[0],start[1],goal[0],goal[1])
 

        # trajectory optimization
        floyd_bspline = Floyd_Bspline(self.map_manager.global_costmap,self.param['bspline_order'], 0.05)
        rx,ry = floyd_bspline.Trajectory_optimize(rx,ry,floyd=self.param['floyd'], Bspline=self.param['bspline'])
        
        self.path_x = [x + self.map_manager.origin_x for x in rx]
        self.path_y = [y + self.map_manager.origin_y for y in ry]


        # publish global path
        self.path_length = len(rx)
        if self.path_length == 0:
            rospy.logwarn("astar didn't find path!")
            return
        
        self.planning_finished = True
        rospy.loginfo("Astar planning finished!")
        

        self.publish_global_path()
        self.map_sub.unregister()
        rospy.loginfo("The global path was published successfully!")


    def run(self):
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            try:
                self.global_planning()     
            except Exception as e:
                rospy.logwarn(e)
            rate.sleep()



if __name__ == '__main__':
    global_planner = Global_planner()
    global_planner.run()


