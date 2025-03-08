from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, PolygonStamped, Point32
from ourplanner.msg import global_path, ellipses, polygons, dynamic_obstacles, pose
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import rospy
import math

class Rviz_drawer():
    def __init__(self) -> None:
        self.global_path_pub = rospy.Publisher("/global_path_rviz",Path,queue_size=5)
        self.middle_path_pub = rospy.Publisher("/middle_path_rviz",Path,queue_size=5)
        self.local_path_pub = rospy.Publisher("/local_path_rviz",Path,queue_size=5)
        self.ellipses_pub = rospy.Publisher("/ellipses_rviz",Marker,queue_size=5)
        self.polygons_pub = rospy.Publisher("/polygons_rviz",PolygonStamped,queue_size=5)
        self.local_state_pub = rospy.Publisher('/local_state_rviz', Marker, queue_size=10)
        self.circles_pub = rospy.Publisher('/local_circles', Marker, queue_size=10)


    def global_path_rviz_show(self, msg:global_path):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"  
        for i in range(msg.length):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = msg.path_x[i]
            pose.pose.position.y = msg.path_y[i]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.global_path_pub.publish(path)



    def middle_path_rviz_show(self, msg:global_path):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"  
        for i in range(msg.length):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = msg.path_x[i]
            pose.pose.position.y = msg.path_y[i]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.middle_path_pub.publish(path)


    def local_path_rviz_show(self, global_states_sol:list):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"  
        for state in global_states_sol:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.local_path_pub.publish(path)


    def local_state_rviz_show(self, start, goal):
        start_marker = self.get_circle_marker(start[0], start[1],0.1,(1,0,0,1),0)
        goal_marker = self.get_circle_marker(goal[0], goal[1],0.1,(0,0,0,1),1)
        self.local_state_pub.publish(start_marker)
        self.local_state_pub.publish(goal_marker)


    def ellipses_rviz_show(self, ellipses_msg:ellipses):

        markers = []
        cnt = 0
        for ellipses in ellipses_msg:
            for ellipse in ellipses:
                cnt = cnt + 1
                markers.append(self.get_ellipse_marker(ellipse,(0.07,0.95,0.75,1), cnt))
        for m in markers:
            self.ellipses_pub.publish(m)


    def polygons_rviz_show(self, polygons_msg:polygons):
        polygons_ = []
        for polygon in polygons_msg:
            polygons_.append(self.get_polygon_marker(polygon))

        for polygon_ in polygons_:
            self.polygons_pub.publish(polygon_)

    def local_circle_show(self, circles):
        if circles == []:
            return
        id = 10
        for circle in circles:
            id = id + 1
            marker = self.get_circle_marker(circle[0][0],circle[0][1],circle[1],(0, 0.5, 0.5, 1),id)
            self.circles_pub.publish(marker)



    @ staticmethod
    def get_circle_marker(x,y,r,color,id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "person_detections"
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for angle in range(360):
            x = r * math.cos(angle * math.pi / 180)
            y = r * math.sin(angle * math.pi / 180)
            p = Point(x, y, 0)
            marker.points.append(p)
        marker.points.append(marker.points[0])
        return marker


    @ staticmethod
    def get_ellipse_marker(ellipse,color,id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ellipses"
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.position.x = ellipse[0]
        marker.pose.position.y = ellipse[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for angle in range(360):
            local_x  = ellipse[2] * math.cos(angle * math.pi / 180)
            local_y  = ellipse[3] * math.sin(angle * math.pi / 180)
            global_x = local_x*math.cos(ellipse[4]) - local_y*math.sin(ellipse[4])
            global_y = local_y*math.cos(ellipse[4]) + local_x*math.sin(ellipse[4])
            p = Point(global_x, global_y, 0)
            marker.points.append(p)
        marker.points.append(marker.points[0])
        return marker
    

    @ staticmethod
    def get_polygon_marker(polygon_points):
        polygon = PolygonStamped()
        polygon.header.frame_id = "map"  
        polygon.header.stamp = rospy.Time.now()  
        points = []
        for i in range(len(polygon_points)):
            points.append(Point32(x=polygon_points[i][0], y=polygon_points[i][1], z=0.0))
        polygon.polygon.points = points
        return polygon


    @ staticmethod
    def creat_circles_marker(circles, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "local_circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Line width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Generate points for the circle
        num_points = 100
        angle_increment = 2 * 3.14159265359 / num_points
        for circle in circles:
            radius = circle[1]
            position = circle[0]
            for i in range(num_points + 1):
                angle = i * angle_increment
                x = radius * math.cos(angle) + position[0]
                y = radius * math.sin(angle) + position[1]
                point = Point()
                point.x = x
                point.y = y
                point.z = 0
                marker.points.append(point)
        
        return marker

