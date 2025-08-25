#!/usr/bin/python
from detection_msgs.msg import tracks as Track_msg
from detection_msgs.msg import Group as Group_msg
from nav_msgs.msg import OccupancyGrid, Path
from ourplanner.msg import ellipses, polygons
from detection_msgs.msg import Groups
from scipy.spatial import ConvexHull
import numpy as np
import rospkg
import rospy
import cv2
import math



class Map_manager:
    def __init__(self) -> None:
        rospack = rospkg.RosPack()
        self.pkg_path  = rospack.get_path('ourplanner')
        self.origin_map_path = self.pkg_path + '/map/global_planner/origin_map.png'
        self.global_costmap_path = self.pkg_path + '/map/global_planner/costmap.png'
        self.middle_costmap_path = self.pkg_path + '/map/middle_planner/costmap.png'
        self.inflate_size = rospy.get_param('map_manager/inflate_size',3)
        self.erode_size = rospy.get_param('map_manager/erode_size',3)
        self.person_inflate_size = rospy.get_param('map_manager/person_inflate_size', 1)
        self.group_inflate_size = rospy.get_param('map_manager/group_inflate_size', 1)

        self.resolution = 0.05
        

    def Map_cb(self, msg:OccupancyGrid):
        rospy.loginfo("got map")
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution

        self.grid_map = msg.data
        self.grid_map_width = msg.info.width
        self.grid_map_height = msg.info.height

        self.Get_origin_map()
        self.Get_global_costmap()

    
    def Get_origin_map(self):
        self.origin_map = self._gridmap2image(self.grid_map_width, self.grid_map_height, self.grid_map)
        cv2.imwrite(self.origin_map_path, self.origin_map)


    def Get_global_costmap(self):
        self.global_costmap = self._inflate_costmap(self.erode_size, self.inflate_size, self.origin_map)
        cv2.imwrite(self.global_costmap_path,self.global_costmap)


    def Get_middle_costmap(self, ellipses_group, polygons, poses):
        global_costmap = self.global_costmap.copy()

        da = self.group_inflate_size
        db = self.group_inflate_size
        for ellipses in ellipses_group: # draw ellipses
            for i in range(len(ellipses)):
                center_coordinates = (int((ellipses[i][1]-self.origin_y)/self.resolution), int((ellipses[i][0] - self.origin_x)/self.resolution))
                axes_length = (int((ellipses[i][2] + da)/self.resolution), int((ellipses[i][3] + db)/self.resolution))
                angle = 90 -  ellipses[i][4] * 180.0 / math.pi 
                color = 0
                thickness = -1
                cv2.ellipse(global_costmap, center_coordinates, axes_length, angle, 0, 360, color, thickness)

        for polygon in polygons:  # draw polygons 
            if polygon == []:
                continue
            points = []
            for i in range(len(polygon)):
                points.append([(polygon[i][1]-self.origin_y)/self.resolution,(polygon[i][0]- self.origin_x)/self.resolution])
            points = np.array(points, np.int32)
            cv2.fillPoly(global_costmap, [points], color=0)

        

        if poses is not None:
            for pose in poses.poses:
                x = int((pose.position.x - self.origin_x)/self.resolution)
                y = int((pose.position.y - self.origin_y)/self.resolution)
                cv2.circle(global_costmap, (y,x), int(self.person_inflate_size/0.05), 0, -1) 

        self.middle_costmap = global_costmap
        cv2.imwrite(self.middle_costmap_path, self.middle_costmap)


    def Set_global_path(self, global_path):
        self.global_path = global_path
        self.global_path_length = len(global_path)


    def _collision(self,index):
        if index >= self.global_path_length:
            return False  
        x = int((self.global_path[index][0] - self.origin_x)/self.resolution)
        y = int((self.global_path[index][1] - self.origin_y)/self.resolution)
        return self.middle_costmap[x][y] < 127 # collided



    def _gridmap2image(self,width,height,data,threshold = 50):
        image = np.empty([width,height],dtype=int)
        for y in range(height):
            for x in range(width):
                if ( data[y*width+x] < 0): # unknow
                    image[x][y] = 0
                else:
                    if( data[y*width+x] > threshold ):
                        image[x][y] = 0
                    else:
                        image[x][y] = 255
        return image
    

    def _inflate_costmap(self, erode_size, inflate_size, costmap):
        costmap = 255 - costmap
        costmap_uint8 = costmap.astype(np.uint8)

        erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_size, erode_size))
        inflate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflate_size, inflate_size))


        eroded_map = cv2.erode(costmap_uint8,erode_kernel,iterations=1)
        inflated_costmap = cv2.dilate(eroded_map, inflate_kernel,iterations=1)

        inflated_costmap = 255 - inflated_costmap
        return inflated_costmap


