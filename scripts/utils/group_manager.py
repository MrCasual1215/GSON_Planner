from detection_msgs.msg import tracks as Track_msg
from detection_msgs.msg import Group as Group_msg
from ourplanner.msg import ellipses, polygons
from detection_msgs.msg import Groups
from scipy.spatial import ConvexHull
import sys
sys.path.insert(0,"/home/sp/planner_ws/src/ourplanner/scripts/utils")
from rviz_drawer import Rviz_drawer
import numpy as np
import rospy
import math

class Group:
    poses:list          # group中所有人的 pose  [x, y]
    ids:list            # group中所有人的 id     
    vels:list           # group中所有人的 velovity  [vx, vy]
    stamps:list         # 记录上一次改变pose的时刻
    person_number:int   # group中的总人数


    def __init__(self, pose_list, id_list, vel_list, stamp) -> None:
        self.initial_pose = pose_list
        self.poses = pose_list
        self.ids = id_list
        self.vels = vel_list
        self.person_number = len(self.poses)
        self.stamps = [ stamp for i in range(self.person_number)]


class Group_manager:
    '''
    通过维护一系列GPT所识别到的group, 即self.groups,来获取middle planning所需要的人群的凸包信息,
    包络椭圆和凸多边形,即self.ellipses_group与self.polygons,并将其发布。
    
    Inputs:
        group_msg               : 获取GPT的分组结果
        track_msg               : 用来更新每个group中人的pose

    Output:
        ellipses_group          ：人群的包络椭圆
        polygons                ：人群的凸多边形
    '''

    groups:list
    group_number:int
    polygons:list
    ellipses_group:list
    GPT_PERCEIVED:bool             # 获取到了GPT信息


    def __init__(self) -> None:
        self.polygons_pub = rospy.Publisher('/polygons', polygons, queue_size=1)
        self.ellipses_pub = rospy.Publisher('/ellipses', ellipses, queue_size=1)
        self.groups = []
        self.polygons = []
        self.ellipses_group = []
        self.rviz_drawer = Rviz_drawer()
        self.GPT_PERCEIVED = True


    def Group_add(self, group_msg:Groups):
        # rospy.loginfo(group_msg)
        rospy.loginfo("Group msg received!!!!!!!!!!!!!!!!!!!!!!!!")
        """
        将GPT识别到的分组结果记录下来 [poses,ids,vels,stamps]

        Args:
            group_msg             : GPT识别得到的结果, 每个group中由poselist, idlist, vellist组成
        """
        if_return = True
        for group in group_msg.group_list:
            print(len(group.group_id_list))
            if len(group.group_id_list) > 1:
                if_return = False

        if if_return:
            return


        # for group in group_msg.group_list:
        #     rospy.loginfo(f"new groups:{group.group_id_list}")

        # for new_group in group_msg.group_list:
        #             new_pose_list, new_id_list, new_vel_list = [], [], []
        #             for pose, id, vx, vy in zip(new_group.group_pose_list.poses, new_group.group_id_list, new_group.group_vel_x_list, new_group.group_vel_y_list):
        #                 new_pose_list.append([pose.position.x, pose.position.y])
        #                 new_id_list.append(id)
        #                 new_vel_list.append([vx, vy])

        #             merged = False
        #             # 遍历已有的groups
        #             for existing_group in self.groups:
        #                 existing_group: Group
        #                 # 检查是否有相同的ID
        #                 if any(id in existing_group.id_list for id in new_id_list):
        #                     # 合并位置信息、ID和速度
        #                     existing_group.poses.extend(new_pose_list)
        #                     existing_group.ids.extend(new_id_list)
        #                     existing_group.vels.extend(new_vel_list)
        #                     existing_group.stamps.extend([ group_msg.header.stamp for i in range(len(new_id_list))])
        #                     existing_group.person_number += len(new_id_list)
        #                     merged = True
        #                     break
                        
        #             if not merged:
        #                 # 如果没有找到相同的ID，创建新的group
        #                 new_group = Group(new_pose_list, new_id_list, new_vel_list, group_msg.header.stamp)
        #                 self.groups.append(new_group)

        # rospy.loginfo("xixi")
        # for group in self.groups:
        #     rospy.loginfo(f"merged groups:{group.ids}")

        groups = []
        for group in group_msg.group_list:
            group:Group_msg # 注意与下方group区别 GPT输出的msg.group 
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
            group:Group # 注意与上方group区别 Group_manager 维护的self.group
            # 找到每个group的凸包 ， 获取凸多边形和椭圆
            if group.person_number > 2:
                self.hull = ConvexHull(group.poses)
                polygon = self.get_polygons(group.poses,self.hull.vertices,0.1)
                ellipses = self.get_ellipses(group.poses,self.hull.vertices,0.5)
            elif group.person_number == 2:  
                # 只有两个人的情况
                polygon = self.get_polygons(group.poses,[0,1],0.1)
                ellipses = self.get_ellipses(group.poses,[0,1],0.5)
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
        只关注已经在groups中的ids, 不考虑单个人(交给local planner)

            1) id处于跟踪状态(in tracks)             : 用tracks来更新人的 pose, vel, stamp 
            2) group中的id丢失                      : 使用最后一次出现的 pose 与 vel 来预测现在这一帧的位置

        Args:
            track_msg            : perception module 所实时得到的人的poses, ids, vels
        """
        
        if not self.GPT_PERCEIVED:
            return
        polygons = []
        ellipses_group = []
        for group in self.groups:
            group:Group # 注意与上方group区别 Group_manager 维护的self.group
            # 更新每一个group的pose
            for index in range(group.person_number):
                if group.ids[index] in track_msg.track_id_list: 
                    # 同一id处于跟踪状态, 更新pose, vel, stamp
                    index_in_tracks = track_msg.track_id_list.index(group.ids[index])  
                    group.poses[index] = [track_msg.track_pose_list.poses[index_in_tracks].position.x, \
                                          track_msg.track_pose_list.poses[index_in_tracks].position.y]
                    group.vels[index] = [track_msg.track_vel_x_list[index_in_tracks], track_msg.track_vel_y_list[index_in_tracks]]
                    group.stamps[index] = track_msg.header.stamp
                else:   
                    # 人口失踪: 用vel更新pose
                    dt = (track_msg.header.stamp - group.stamps[index]).to_sec()
                    dx = group.vels[index][0]*dt
                    dy = group.vels[index][1]*dt
                    # group.poses[index] =  [group.poses[index][0] + dx, group.poses[index][1] + dy] 

            # 找到每个group的凸包 ， 获取凸多边形和椭圆
            if group.person_number > 2:
                self.hull = ConvexHull(group.poses)
                polygon = self.get_polygons(group.poses,self.hull.vertices,0.1)
                ellipses = self.get_ellipses(group.poses,self.hull.vertices,0.5)
            elif group.person_number == 2:  
                # 只有两个人的情况
                polygon = self.get_polygons(group.poses,[0,1],0.1)
                ellipses = self.get_ellipses(group.poses,[0,1],0.5)
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
        将每一个group的包络椭圆和凸多边形 publish
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
        self.ellipses_msg = ellipses_msg # 记录msg 
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
        self.polygons_msg = polygons_msg # 记录msg 
        self.polygons_pub.publish(polygons_msg)

        self.rviz_drawer.ellipses_rviz_show(self.ellipses_group)
        self.rviz_drawer.polygons_rviz_show(self.polygons)




    @ staticmethod
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



    @ staticmethod
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
    

