from detection_msgs.msg import tracks as Track_msg
from detection_msgs.msg import Group as Group_msg
from ourplanner.msg import ellipses, polygons
from detection_msgs.msg import Groups
from scipy.spatial import ConvexHull
import sys
from utils.rviz_drawer import Rviz_drawer
import numpy as np
import rospy
import math

class Group:
    poses:list           
    ids:list             
    vels:list           
    stamps:list         
    person_number:int   


    def __init__(self, pose_list, id_list, vel_list, stamp) -> None:
        self.initial_pose = pose_list
        self.poses = pose_list
        self.ids = id_list
        self.vels = vel_list
        self.person_number = len(self.poses)
        self.stamps = [ stamp for i in range(self.person_number)]


class Group_manager:
    '''
    By maintaining a series of groups recognized by GPT (i.e., self.groups), 
    this module obtains the crowd convex hull information required for middle planning, 
    including the enclosing ellipses and convex polygons (self.ellipses_group and self.polygons), 
    and publishes them.

    Inputs:
        group_msg   : Grouping results obtained from GPT
        track_msg   : Used to update the pose of each person within a group

    Outputs:
        ellipses_group  : Enclosing ellipses of the crowd
        polygons        : Convex polygons of the crowd
    '''


    groups:list
    group_number:int
    polygons:list
    ellipses_group:list
    GPT_PERCEIVED:bool          


    def __init__(self) -> None:
        self.polygons_pub = rospy.Publisher('/polygons', polygons, queue_size=1)
        self.ellipses_pub = rospy.Publisher('/ellipses', ellipses, queue_size=1)
        self.p_step = rospy.get_param('group_manager/predicted_steps', 0.1)
        self.timestep = rospy.get_param('timestep', 0.1)
        self.poses = None
        self.groups = []
        self.polygons = []
        self.ellipses_group = []
        self.rviz_drawer = Rviz_drawer()
        self.GPT_PERCEIVED = True


    def Group_add(self, group_msg:Groups):
        # rospy.loginfo("Group msg received!")
        """
        Record the group results recognized by GPT [poses, ids, vels, stamps].

        Args:
            group_msg   : Results obtained from GPT, where each group consists of a poselist, idlist, and vellist
        """

        if all(len(group.group_id_list) <= 1 for group in group_msg.group_list):
            return

        groups = []
        for group in group_msg.group_list:
            group:Group_msg 
            pose_list, id_list, vel_list = [], [], []
            for pose, id, vx, vy in zip(group.group_pose_list.poses, group.group_id_list, group.group_vel_x_list, group.group_vel_y_list):
                pose_list.append([pose.position.x,pose.position.y])
                id_list.append(id)
                vel_list.append([vx, vy])
            group = Group(pose_list, id_list, vel_list, group_msg.header.stamp)
            groups.append(group)
        self.groups = groups

        self.group_number = len(self.groups)
        self.GPT_PERCEIVED = True


        polygons = []
        ellipses_group = []
        for group in self.groups:
            group:Group 
            poses = group.poses
            for n in range(self.p_step+1):
                poses = [[pose[0] + group.vels[i][0]*n*self.timestep, pose[1] + group.vels[i][1]*n*self.timestep] for i, pose in enumerate(poses)]
                if group.person_number > 2:
                    self.hull = ConvexHull(group.poses)
                    polygon = self.get_polygons(poses,self.hull.vertices,0.1)
                    ellipses = self.get_ellipses(poses,self.hull.vertices,0.5)
                elif group.person_number == 2:  
                    polygon = self.get_polygons(poses,[0,1],0.1)
                    ellipses = self.get_ellipses(poses,[0,1],0.5)
                else:
                    polygon = []
                    ellipses = []

                polygons.append(polygon)
                ellipses_group.append(ellipses)
        
        self.polygons = polygons
        self.ellipses_group = ellipses_group
        self.Group_pub()



    def Group_update(self, track_msg:Track_msg):
        """
        Focus only on the IDs already present in groups, ignoring single individuals 
        (these are handled by the local planner).

            1) If an ID is in the tracking state (in tracks): 
            use tracks to update the person's pose, velocity, and timestamp.  
            2) If an ID in the group is lost: 
            use the last known pose and velocity to predict the current position in this frame.  

        Args:
            track_msg   : Real-time poses, IDs, and velocities of people obtained from the perception module
        """


        self.poses = track_msg.track_pose_list
        
        if not self.GPT_PERCEIVED:
            return
        polygons = []
        ellipses_group = []
        for group in self.groups:
            group:Group 
            # update each person's pose in group
            for index in range(group.person_number):
                if group.ids[index] in track_msg.track_id_list: 
                    # update pose, vel, stamp
                    index_in_tracks = track_msg.track_id_list.index(group.ids[index])  
                    group.poses[index] = [track_msg.track_pose_list.poses[index_in_tracks].position.x, \
                                          track_msg.track_pose_list.poses[index_in_tracks].position.y]
                    group.vels[index] = [track_msg.track_vel_x_list[index_in_tracks], track_msg.track_vel_y_list[index_in_tracks]]
                    group.stamps[index] = track_msg.header.stamp
                else:   
                    # using last pose and vel to predict current pose
                    dt = (track_msg.header.stamp - group.stamps[index]).to_sec()
                    dx = group.vels[index][0]*dt
                    dy = group.vels[index][1]*dt
                    # group.poses[index] =  [group.poses[index][0] + dx, group.poses[index][1] + dy] 

            poses = group.poses
            for n in range(self.p_step+1):
                poses = [[pose[0] + group.vels[i][0]*n*self.timestep, pose[1] + group.vels[i][1]*n*self.timestep] for i, pose in enumerate(poses)]
                if group.person_number > 2:
                    self.hull = ConvexHull(group.poses)
                    polygon = self.get_polygons(poses,self.hull.vertices,0.1)
                    ellipses = self.get_ellipses(poses,self.hull.vertices,0.5)
                elif group.person_number == 2:  
                    # only two people
                    polygon = self.get_polygons(poses,[0,1],0.1)
                    ellipses = self.get_ellipses(poses,[0,1],0.5)
                else:
                    polygon = []
                    ellipses = []

                polygons.append(polygon)
                ellipses_group.append(ellipses)
        
        self.polygons = polygons
        self.ellipses_group = ellipses_group
        # 每次update后publish
        self.Group_pub()



    def Group_pub(self):
        """
        publish self.ellipses_group and self.polygons
        """

        # publish ellipses
        n, x, y, a, b, theta = 0, [], [], [], [], []
        for _ellipses in self.ellipses_group:
            n = n + len(_ellipses)
            for ellipse in _ellipses:
                x.append(ellipse[0]),y.append(ellipse[1])
                a.append(ellipse[2]),b.append(ellipse[3]) ,theta.append(ellipse[4])

        ellipses_msg = ellipses()
        ellipses_msg.length = n
        ellipses_msg.x = x
        ellipses_msg.y = y
        ellipses_msg.a = a
        ellipses_msg.b = b
        ellipses_msg.theta = theta
        self.ellipses_msg = ellipses_msg  
        self.ellipses_pub.publish(ellipses_msg)


        # pulish polygons
        n = len(self.polygons)
        x, y, index = [], [], []
        for polygon in self.polygons:
            index.append(len(polygon))
            for point in polygon:
                x.append(point[0])
                y.append(point[1])


        polygons_msg = polygons()
        polygons_msg.number = n
        polygons_msg.index = index
        polygons_msg.points_x = x
        polygons_msg.points_y = y
        self.polygons_msg = polygons_msg 
        self.polygons_pub.publish(polygons_msg)

        self.rviz_drawer.ellipses_rviz_show(self.ellipses_group)
        self.rviz_drawer.polygons_rviz_show(self.polygons)




    @staticmethod
    def get_ellipses(detections,order,r) -> list:
        ordered_points = []
        for i in range(len(order)):
            ordered_points.append(np.array(detections[order[i]]))
        n = len(ordered_points)
        length = len(ordered_points)

        if n == 2:
            n = 1

        ellipses = []
        for i in range(n):
            p1 = ordered_points[i]              # current point
            p2 = ordered_points[(i + 1) % length]    # next point
            centr = (p1 + p2)/2.0
            theta = math.atan2((p2[1]-p1[1]),(p2[0]-p1[0]))
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

            k = 0.1
            c = dist/2.0
            b = k * dist + r 
            a = math.sqrt(b**2 + c**2) + 0.3
            ellipse = np.array([centr[0], centr[1], a, b, theta])
            ellipses.append(ellipse)
        return ellipses



    @staticmethod
    def get_polygons(detections,order,r) -> list:
        ordered_points = []
        for i in range(len(order)):
            ordered_points.append(np.array(detections[order[i]]))
        
        new_points = []
        if len(order) == 2:
            p0 = ordered_points[0]
            p1 = ordered_points[1]

            v1 = (p1 - p0) / np.linalg.norm(p1 - p0)
            n1 = np.array([v1[1], -v1[0]])

            a1 = p1 + v1*r + n1*r
            a2 = p1 + v1*r - n1*r
            a3 = p0 - v1*r - n1*r
            a4 = p0 - v1*r + n1*r
            return [a1,a2,a3,a4] 

        n = len(ordered_points)
        for i in range(n):
            p0 = ordered_points[(i + n -1) % n] # previous point
            p1 = ordered_points[i]              # current point
            p2 = ordered_points[(i + 1) % n]    # next point
            # Calculate the vectors
            v1 = (p1 - p0) / np.linalg.norm(p1 - p0)
            v2 = (p2 - p1) / np.linalg.norm(p2 - p1)
            # Calculate the normals
            n1 = np.array([v1[1], -v1[0]])
            n2 = np.array([v2[1], -v2[0]])
            n3 = (n1 + n2) / np.linalg.norm(n1 + n2)
            cos = math.sqrt( (n1[0]*n2[0]+n1[1]*n2[1] + 1)/2.0)
            cos = max(0.5,cos)
            new_point = p1 + n3 * r / cos
            new_points.append(new_point)
        return new_points
    

