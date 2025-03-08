from detection_msgs.msg import tracks as Track_msg
from detection_msgs.msg import Group as Group_msg
from nav_msgs.msg import OccupancyGrid, Path
from ourplanner.msg import ellipses, polygons
from detection_msgs.msg import Groups
from scipy.spatial import ConvexHull
import numpy as np
import rospy
import cv2
import math



class Map_manager:
    def __init__(self) -> None:
        self.pkg_path = rospy.get_param("planner_pkg_path")
        self.origin_map_path = self.pkg_path + rospy.get_param("origin_map_path")
        self.global_costmap_path = self.pkg_path +rospy.get_param("global_costmap_path")
        self.middle_costmap_path = self.pkg_path +rospy.get_param("middle_costmap_path")
        self.resolution = 0.05
        


    def Get_global_costmap(self):
        self.Get_origin_map()
        self.global_costmap = self._inflate_costmap(0.4, self.origin_map)

    def Map_cb(self, msg:OccupancyGrid):
        rospy.loginfo("got fxxking map")


        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution

        self.grid_map = msg.data
        self.grid_map_width = msg.info.width
        self.grid_map_height = msg.info.height

    
    def Get_origin_map(self):
        # map = cv2.imread("/home/sp/planner_ws/src/ourplanner/map/global_planner/testmap.png",cv2.IMREAD_GRAYSCALE)
        # map = cv2.transpose(map)
        # map = cv2.flip(map,1)
        # _, self.origin_map = cv2.threshold(map,220,255,cv2.THRESH_BINARY)
        rospy.loginfo("got fxxking origin map!")
        self.origin_map = self._gridmap2image(self.grid_map_width, self.grid_map_height, self.grid_map)
        cv2.imwrite(self.origin_map_path, self.origin_map)


    def Get_global_costmap(self):
        self.Get_origin_map()
        self.global_costmap = self._inflate_costmap(0.4, self.origin_map)
        rospy.loginfo("got fxxking global costmap!")
        cv2.imwrite(self.global_costmap_path,self.global_costmap)


    def Get_middle_costmap(self, ellipses_group, polygons):
        self.global_costmap = cv2.imread(self.origin_map_path, cv2.IMREAD_GRAYSCALE)
        global_costmap = self.global_costmap.copy()

        da = 0.4
        db = 0.3
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
    

    def _inflate_costmap(self,inflation_radius,costmap):
        costmap = 255 - costmap
        costmap_uint8 = costmap.astype(np.uint8)



        kernel = np.ones((1,1),np.uint8)
        eroded_map = cv2.erode(costmap_uint8,kernel,iterations=1)


        # inflation_radius_in_cells = int(inflation_radius / self.resolution)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*inflation_radius_in_cells+1, 2*inflation_radius_in_cells+1))
        inflated_costmap = cv2.dilate(eroded_map, kernel,iterations=1)


        inflated_costmap = 255 - inflated_costmap
        return inflated_costmap


