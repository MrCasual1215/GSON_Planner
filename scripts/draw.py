#! /usr/bin/python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from detection_msgs.msg import tracks
from ourplanner.msg import global_path
from std_msgs.msg import Header
import tf2_ros

rx =  [-19.24859619140625, -17.6615467951808, -16.31600491577227, -15.211970553180654, -14.323413879381684, -13.469512302064732, -12.615610724747782, -11.761709147430828, -10.908411011551364, -10.056509003494723, -9.206006693920301, -8.356796070381446, -7.507843426983765, -6.658890783586083, -5.809938140188401, -4.960985496790721, -4.112032853393039, -3.263080209995357, -2.41605180945278, -1.5976970030597077, -0.8165917373678993, -0.07273601237735672, 0.6453681168732837, 1.3626170270772113, 2.079865937281137, 2.797114847485066, 3.5143637576889937, 4.231612667892922, 4.94886157809685, 5.683124942381894, 6.481362654011954, 7.346297025640004, 8.276864653885973, 9.229933897654222, 10.183003141422471, 11.136072385190722, 12.078064236759158, 12.969865928607481, 13.808058512525871, 14.59294969385321, 15.35647244886919, 16.11999520388517, 16.883517958901148, 17.647040713917132, 18.410563468933113, 19.17408622394909, 19.93759517493234, 20.68952254245498, 21.4198051866567, 22.128443107537517, 22.81814189551255, 23.505135093072457, 24.192128290632365, 24.87912148819227, 25.56611468575218, 26.25310788331209, 26.940101080872005, 27.593787036833874, 27.94400701379142, 27.93154813779259, 27.556410408837365, 27.028766680537352, 26.662699265329707, 26.461234652306423, 26.413543758191892, 26.39574187031304, 26.37274275885688, 26.344546423823402, 26.312906265258796, 26.281183242797855, 26.249460220336914, 26.218001113742144, 26.191358471704913, 26.17118176838875, 26.15747100379367, 26.14840763265437, 26.13954632210009, 26.130685011545822, 26.121823700991545, 26.11296239043727, 26.104101079882994, 26.095239769328725, 26.08637845877445, 26.077517148220178, 26.0686558376659, 26.059794527111627, 26.05093321655736, 26.042071906003088, 26.033210595448814, 26.024349284894544, 26.015487974340264, 26.006626663785994, 25.997765353231717, 25.988904042677444, 25.980042732123174, 25.971181421568904, 25.96196047990447, 25.950503431915106, 25.93654015467807, 25.92007064819336]
ry =  [6.885887622833252, 6.232212068255901, 5.678008880679454, 5.223278060103906, 4.857298413912455, 4.505592544873555, 4.153886675834656, 3.802180806795755, 3.476649604771966, 3.2116763601618366, 3.007415952651844, 2.8591832717259718, 2.7221406481482764, 2.58509802457058, 2.4480554009928843, 2.3110127774151885, 2.1739701538374927, 2.036927530259797, 1.8977949006826746, 1.7275186014597868, 1.5167839144949262, 1.2655908397880937, 0.9864278082494377, 0.7063358851554812, 0.42624396206152526, 0.14615203896756873, -0.13393988412638794, -0.4140318072203446, -0.6941237303143013, -0.98513860361019, -1.3172237696653493, -1.6921269005120876, -2.1091653117627835, -2.540649324655536, -2.9721333375482883, -3.403617350441041, -3.831376404391177, -4.242258175230959, -4.635112984274391, -5.010044302603213, -5.377790129144588, -5.745535955685963, -6.113281782227334, -6.481027608768709, -6.8487734353100835, -7.216519261851457, -7.584264366082571, -7.951403452004603, -8.31740995543719, -8.68228387638034, -9.046166787645232, -9.409908126098944, -9.773649464552653, -10.137390803006362, -10.501132141460074, -10.864873479913786, -11.228614818367499, -11.612447972322485, -12.179339889733562, -12.965009353714121, -13.969456364264143, -15.05950465784696, -16.036135626233325, -16.897431531228325, -17.655072095454717, -18.44267334571701, -19.298077583313002, -20.22128480824266, -21.189420160621125, -22.15863655552721, -23.127852950433308, -24.093468788776736, -24.993374469851666, -25.805066515141494, -26.528544924646216, -27.1886197834304, -27.845937966096287, -28.503256148762176, -29.160574331428073, -29.817892514093963, -30.475210696759856, -31.13252887942575, -31.789847062091646, -32.447165244757535, -33.104483427423425, -33.761801610089314, -34.41911979275522, -35.076437975421115, -35.733756158087004, -36.391074340752894, -37.048392523418784, -37.70571070608467, -38.36302888875057, -39.02034707141646, -39.677665254082356, -40.334983436748246, -41.018978493710634, -41.86884442685442, -42.9046185328734, -44.12630081176758]


p1 = [1, 2]
p2 = [3, 4]
robot_pose = [5,6]

class RvizShower:
    def __init__(self) -> None:
        rospy.Subscriber("/global_path", global_path,self.Global_path_cb,queue_size=1)
        rospy.Subscriber("/middle_path", global_path,self.Middle_path_cb,queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.Odom_cb,queue_size=1)

        self.middle_path_pub = rospy.Publisher("/m_path",Path,queue_size=5)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.number_marker_pub = rospy.Publisher('/number', Marker, queue_size=1)

        self.marker_array = MarkerArray()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.last_group = []
        self.nvsheng = []

    def Odom_cb(self, msg:Odometry):
        self.time = msg.header.stamp.to_sec()



    def Global_path_cb(self,msg:global_path):
        self.global_path = [[x, y] for x,y in zip(msg.path_x, msg.path_y)]


    def Middle_path_cb(self,msg:global_path):
        try:
            self.middle_path = [[x, y] for x,y in zip(msg.path_x, msg.path_y)]
            if math.fabs( (self.middle_path[0][0] - self.global_path[0][0])**2 + (self.middle_path[0][1] - self.global_path[0][1])**2) > 0.1:
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
            else:
                path = Path()
                path.header.stamp = rospy.Time.now()
                path.header.frame_id = "map"  
                self.middle_path_pub.publish(path) 
        except Exception as e:
            return


    def GetRobotPose(self):
        try:
            pose = self.tf_buffer.lookup_transform("map", "laser_frame",rospy.Time(0))
            tx  = pose.transform.translation.x
            ty  = pose.transform.translation.y
            tz  = pose.transform.translation.z
            robot_pose = [tx, ty]
            # rospy.loginfo("haha")
            self.marker_array.markers.append(create_cylinder(robot_pose[0], robot_pose[1], 0.3, (0.96, 0.88, 0.50, 1.0), 0.3, 0.6, 100))
            self.marker_pub.publish(self.marker_array)
            self.robot_pose = robot_pose
        except Exception as e:
            return

    def number_publish(self, x, y, z, a, id):
        marker = Marker()

        # 填充Marker消息的必要字段
        marker.header = Header(stamp=rospy.Time.now(), frame_id='map')
        marker.ns = "numbers"
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # 设置要显示的文本为数字
        marker.text = str(id)  # 可以将此数字动态更新

        # 设置文本的坐标
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # 文本的大小
        marker.scale.z = 1.0

        # 设置文本的颜色
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = a

        self.number_marker_pub.publish(marker)


def publish_paths(path_pub, rx, ry):
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    
    
    for j in range(len(rx)):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = rx[j]
        pose.pose.position.y = ry[j]
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    path_pub.publish(path)



def create_cylinder(x, y, z, color, radius, height, id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cylinders"
    marker.id = id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = radius * 2  # 直径
    marker.scale.y = radius * 2  # 直径
    marker.scale.z = height
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker

def create_circle(x, y, z, radius, id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "circle"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = 0.2  # 线条宽度
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    
    # 创建圆形的点
    for i in range(36):
        angle = i * 10 * math.pi / 180
        point = Point()  # 已正确导入 Point
        point.x =  radius * math.cos(angle)
        point.y =  radius * math.sin(angle)
        point.z = z
        marker.points.append(point)

    # 闭合圆形
    marker.points.append(marker.points[0])

    return marker

def get_ellipses(ordered_points,r) -> list:
    length = len(ordered_points)
    ellipses = []
    for i in range(length):
        p1 = ordered_points[i]              # current point
        p2 = ordered_points[(i + 1) % length]    # next point
        centr = (p1 + p2)/2.0
        theta = math.atan2((p2[1]-p1[1]),(p2[0]-p1[0]))
        dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

        k = 0.1
        c = dist/3.0
        b = k * dist + r + 0.3
        a = math.sqrt(b**2 + c**2) + 0.5
        ellipse = np.array([centr[0], centr[1], a, b, theta])
        ellipses.append(ellipse)
    return ellipses


def create_ellipse_marker(x, y, a, b, theta, frame_id="map", ns="ellipse", marker_id=0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.2  # 线条宽度
    marker.color.r = 0.66
    marker.color.g = 0.86
    marker.color.b = 0.78
    marker.color.a = 1.0

    # 生成椭圆上的点
    num_points = 100  # 椭圆上点的数量
    for i in range(num_points + 1):
        angle = 2 * math.pi * i / num_points  # 从0到2pi的角度
        x_local = a * math.cos(angle)  # 椭圆方程的x
        y_local = b * math.sin(angle)  # 椭圆方程的y

        # 旋转矩阵应用
        x_rotated = x_local * math.cos(theta) - y_local * math.sin(theta)
        y_rotated = x_local * math.sin(theta) + y_local * math.cos(theta)

        # 将椭圆的点添加到 Marker
        point = Point()
        point.x = x + x_rotated
        point.y = y + y_rotated
        point.z = 0  # 在平面上绘制，z = 0
        marker.points.append(point)
    return marker


def main():
    rospy.init_node('path_and_shape_publisher', anonymous=True)
    rvizShower = RvizShower()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ellipsis = get_ellipses(np.array([p1,p2]),0.2)
        marker_array = MarkerArray()
        for i, ellipse in enumerate(ellipsis):
            marker_array.markers.append(create_ellipse_marker(ellipse[0],ellipse[1],ellipse[2],ellipse[3],ellipse[4],marker_id=i))
        
        marker_array.markers.append(create_cylinder(robot_pose[0], robot_pose[1], 0.3, (0.96, 0.88, 0.50, 1.0), 0.3, 0.6, 100))
        marker_array.markers.append(create_cylinder(p1[0], p1[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, id))
        marker_array.markers.append(create_cylinder(p2[0], p2[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, id))
        
        publish_paths(rvizShower.middle_path_pub, rx, ry)


        
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
