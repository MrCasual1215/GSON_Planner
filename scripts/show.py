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


fxxk_time = [1726930256.189891, 1726930256.2416542, 1726930256.2934172, 1726930256.3451803, 1726930256.396943, 1726930256.4487062, 1726930256.5004692, 1726930256.5522323, 1726930256.6039953, 1726930256.6557584, 1726930256.7075214, 1726930256.7592845, 1726930256.8110473, 1726930256.8628104, 1726930256.9145734, 1726930256.9663365, 1726930257.0866723, 1726930257.1552448, 1726930257.2238173, 1726930257.29239, 1726930257.3609629, 1726930257.4295354, 1726930257.498108, 1726930257.5666807, 1726930257.6352534, 1726930257.703826, 1726930257.7723985, 1726930257.8409712, 1726930257.909544, 1726930258.0501182, 1726930258.12212, 1726930258.1941216, 1726930258.2661233, 1726930258.3381248, 1726930258.4101264, 1726930258.4821281, 1726930258.5541298, 1726930258.6261315, 1726930258.6981332, 1726930258.770135, 1726930258.8421366, 1726930258.9141383, 1726930258.98614, 1726930259.0581417, 1726930259.1301432, 1726930259.2021449, 1726930259.2741466, 1726930259.3461483, 1726930259.4810092, 1726930259.5438683, 1726930259.6067276, 1726930259.669587, 1726930259.7324462, 1726930259.7953053, 1726930259.8581645, 1726930259.9210238, 1726930259.983883, 1726930260.0467422, 1726930260.1096015, 1726930260.1724608, 1726930260.2353199, 1726930260.2981791, 1726930260.3610384, 1726930260.4238975, 1726930260.4867568, 1726930260.549616, 1726930260.6124754, 1726930260.6753345, 1726930260.8169005, 1726930260.8956075, 1726930260.9743142, 1726930261.0530212, 1726930261.131728, 1726930261.210435, 1726930261.2891417, 1726930261.3678484, 1726930261.4465554, 1726930261.525262, 1726930261.603969, 1726930261.6826758, 1726930261.7613826, 1726930261.8400896, 1726930261.9187963, 1726930261.9975033, 1726930262.07621, 1726930262.154917, 1726930262.2336237, 1726930262.3123305, 1726930262.3910375, 1726930262.4697442, 1726930262.5484512, 1726930262.627158, 1726930262.7058647, 1726930262.7845716, 1726930262.8632784, 1726930262.9419854, 1726930263.020692, 1726930263.099399, 1726930263.2409742, 1726930263.3038428, 1726930263.3667111, 1726930263.4295795, 1726930263.4924479, 1726930263.5553164, 1726930263.6181848, 1726930263.6810532, 1726930263.7439218, 1726930263.80679, 1726930263.8696585, 1726930263.9325268, 1726930263.9953954, 1726930264.1359043, 1726930264.2135448, 1726930264.2911854, 1726930264.368826, 1726930264.4464667, 1726930264.5241072, 1726930264.6017478, 1726930264.6793883, 1726930264.7570288, 1726930264.8346694, 1726930264.91231, 1726930264.9899504, 1726930265.0675912, 1726930265.1452317, 1726930265.2228723, 1726930265.3005128, 1726930265.4677525, 1726930265.5573514, 1726930265.6469505, 1726930265.7365496, 1726930265.8261485, 1726930265.9157476, 1726930266.0053468, 1726930266.0949457, 1726930266.1845448, 1726930266.274144, 1726930266.3637428, 1726930266.453342, 1726930266.542941, 1726930266.6325402, 1726930266.7221391, 1726930266.8117383, 1726930266.9013374, 1726930266.9909363, 1726930267.0805354, 1726930267.1701345, 1726930267.2597334, 1726930267.3493326, 1726930267.4389317, 1726930267.5285306, 1726930267.696025, 1726930267.7739203, 1726930267.8518155, 1726930267.9297106, 1726930268.007606, 1726930268.0855012, 1726930268.1633964, 1726930268.2412918, 1726930268.319187, 1726930268.397082, 1726930268.4749773, 1726930268.5528727, 1726930268.6307678, 1726930268.708663, 1726930268.7865584, 1726930268.8644536, 1726930268.9423487, 1726930269.0202441, 1726930269.0981393]
fxxk_pose = [[6.754501062280991, -2.14093279838562], [6.847646629109102, -2.1722564697265625], [6.940792195937213, -2.203580141067505], [7.033937762765324, -2.2349038124084473], [7.127083329593434, -2.2662274837493896], [7.220228896421545, -2.297551155090332], [7.313374463249655, -2.3288748264312744], [7.406520030077766, -2.360198497772217], [7.499665596905877, -2.391522169113159], [7.5928111637339875, -2.4228458404541016], [7.685956730562098, -2.454169511795044], [7.779102297390208, -2.4854931831359863], [7.872247864218319, -2.5168168544769287], [7.965393431046429, -2.548140525817871], [8.05853899787454, -2.5794641971588135], [8.15168456470265, -2.610787868499756], [8.335537569863456, -2.6818889379501343], [8.42624500819615, -2.7216663360595703], [8.516952446528844, -2.7614437341690063], [8.607659884861537, -2.8012211322784424], [8.698367323194232, -2.8409985303878784], [8.789074761526924, -2.8807759284973145], [8.87978219985962, -2.9205533266067505], [8.970489638192314, -2.9603307247161865], [9.061197076525007, -3.0001081228256226], [9.151904514857701, -3.0398855209350586], [9.242611953190394, -3.0796629190444946], [9.333319391523089, -3.1194403171539307], [9.424026829855784, -3.1592177152633667], [9.604775667190552, -3.2336785316467287], [9.694817066192627, -3.268361949920654], [9.784858465194702, -3.30304536819458], [9.874899864196777, -3.3377287864685057], [9.964941263198853, -3.3724122047424316], [10.054982662200928, -3.4070956230163576], [10.145024061203003, -3.441779041290283], [10.235065460205078, -3.476462459564209], [10.325106859207153, -3.5111458778381346], [10.415148258209229, -3.5458292961120605], [10.505189657211304, -3.5805127143859865], [10.595231056213379, -3.615196132659912], [10.685272455215454, -3.649879550933838], [10.77531385421753, -3.6845629692077635], [10.865355253219604, -3.7192463874816895], [10.95539665222168, -3.7539298057556154], [11.045438051223755, -3.788613224029541], [11.13547945022583, -3.823296642303467], [11.225520849227905, -3.8579800605773924], [11.41019158136277, -3.917543683733259], [11.504820914495559, -3.9424238886151994], [11.599450247628349, -3.96730409349714], [11.694079580761137, -3.9921842983790805], [11.788708913893927, -4.017064503261021], [11.883338247026716, -4.041944708142962], [11.977967580159506, -4.066824913024902], [12.072596913292294, -4.091705117906843], [12.167226246425084, -4.116585322788784], [12.261855579557873, -4.141465527670724], [12.356484912690663, -4.1663457325526645], [12.451114245823451, -4.191225937434605], [12.545743578956241, -4.216106142316546], [12.64037291208903, -4.240986347198486], [12.73500224522182, -4.265866552080427], [12.829631578354608, -4.290746756962368], [12.924260911487398, -4.315626961844308], [13.018890244620186, -4.3405071667262485], [13.113519577752976, -4.365387371608189], [13.208148910885765, -4.39026757649013], [13.392042467671056, -4.45260347858552], [13.481306691323557, -4.49005917579897], [13.570570914976058, -4.52751487301242], [13.65983513862856, -4.56497057022587], [13.749099362281061, -4.602426267439319], [13.838363585933562, -4.639881964652769], [13.927627809586063, -4.677337661866218], [14.016892033238564, -4.7147933590796685], [14.106156256891065, -4.752249056293119], [14.195420480543568, -4.789704753506568], [14.284684704196069, -4.827160450720018], [14.37394892784857, -4.864616147933468], [14.463213151501071, -4.902071845146917], [14.552477375153572, -4.939527542360367], [14.641741598806075, -4.976983239573817], [14.731005822458574, -5.014438936787267], [14.820270046111077, -5.051894634000717], [14.909534269763578, -5.089350331214167], [14.998798493416079, -5.126806028427616], [15.08806271706858, -5.164261725641066], [15.17732694072108, -5.201717422854516], [15.266591164373583, -5.239173120067965], [15.355855388026084, -5.2766288172814155], [15.445119611678585, -5.314084514494866], [15.534383835331086, -5.351540211708315], [15.623648058983587, -5.388995908921765], [15.712912282636088, -5.426451606135215], [15.802176506288589, -5.463907303348664], [15.891440729941092, -5.501363000562114], [15.980704953593593, -5.5388186977755645], [16.152067729404994, -5.620967830930438], [16.234166281563894, -5.665661266871861], [16.316264833722794, -5.710354702813285], [16.398363385881698, -5.7550481387547086], [16.480461938040598, -5.7997415746961325], [16.5625604901995, -5.8444350106375555], [16.6446590423584, -5.8891284465789795], [16.7267575945173, -5.933821882520403], [16.8088561466762, -5.9785153184618265], [16.8909546988351, -6.02320875440325], [16.973053250994003, -6.067902190344674], [17.055151803152903, -6.112595626286097], [17.137250355311803, -6.157289062227521], [17.31350090924431, -6.233020670273724], [17.407652911017923, -6.264058842378504], [17.50180491279153, -6.295097014483283], [17.595956914565143, -6.326135186588063], [17.69010891633875, -6.357173358692842], [17.784260918112363, -6.388211530797622], [17.87841291988597, -6.419249702902401], [17.972564921659583, -6.450287875007181], [18.06671692343319, -6.48132604711196], [18.160868925206803, -6.51236421921674], [18.25502092698041, -6.543402391321519], [18.349172928754022, -6.574440563426299], [18.44332493052763, -6.605478735531078], [18.537476932301242, -6.636516907635857], [18.63162893407485, -6.667555079740636], [18.725780935848462, -6.698593251845416], [18.913858795166014, -6.76230712890625], [19.00778465270996, -6.794982833862305], [19.101710510253906, -6.827658538818359], [19.195636367797853, -6.860334243774414], [19.289562225341797, -6.893009948730469], [19.38348808288574, -6.925685653686523], [19.477413940429688, -6.958361358642578], [19.571339797973632, -6.9910370635986325], [19.66526565551758, -7.023712768554687], [19.759191513061523, -7.056388473510742], [19.853117370605467, -7.0890641784667965], [19.947043228149415, -7.121739883422851], [20.04096908569336, -7.154415588378907], [20.134894943237306, -7.187091293334961], [20.22882080078125, -7.219766998291016], [20.322746658325194, -7.252442703247071], [20.41667251586914, -7.285118408203125], [20.510598373413085, -7.31779411315918], [20.604524230957033, -7.350469818115235], [20.698450088500977, -7.383145523071289], [20.79237594604492, -7.415821228027344], [20.886301803588868, -7.4484969329833985], [20.980227661132812, -7.481172637939453], [21.07415351867676, -7.513848342895508], [21.255985460783307, -7.592550729450426], [21.343891545345908, -7.63857741104929], [21.431797629908512, -7.684604092648155], [21.519703714471113, -7.730630774247019], [21.607609799033717, -7.776657455845883], [21.695515883596318, -7.822684137444747], [21.783421968158923, -7.868710819043611], [21.871328052721527, -7.914737500642476], [21.959234137284128, -7.9607641822413395], [22.047140221846732, -8.006790863840203], [22.135046306409333, -8.052817545439067], [22.222952390971937, -8.098844227037931], [22.31085847553454, -8.144870908636795], [22.398764560097142, -8.190897590235659], [22.486670644659746, -8.236924271834525], [22.574576729222347, -8.282950953433389], [22.66248281378495, -8.328977635032253], [22.750388898347552, -8.375004316631117], [22.838294982910156, -8.42103099822998]]



ellipse_1 = [1726930199.577490091, 1726930208.858113765]
ellipse_2 = [1726930243.338109016, 1726930267.218119382]
ellipse_3 = [1726930313.098274278, 1726930339.673863172]
ellipse_4 = [1726930351.318157196, 1726930389.358131408]

ellipse_starts = [1726930199.577490091, 1726930243.338109016, 1726930313.298274278, 1726930351.318157196]
ellipse_ends = [1726930208.85, 1726930267.22, 1726930339.67, 1726930389.35]

id_list = [3, 2, 4, 5, 6, 7]
poses =  [ [7.23, -0.06], [6.57, -1.38], [28.40, -13.69], [26.89, -13.04], [28.65, -26.32], [25.43, -26.33]]
id_list_predefine = [7,8, 24,25 ,36, 37, 52, 59]


class RvizShower:
    def __init__(self) -> None:
        rospy.Subscriber("/global_path", global_path,self.Global_path_cb,queue_size=1)
        rospy.Subscriber("/middle_path", global_path,self.Middle_path_cb,queue_size=1)
        rospy.Subscriber("/tracker", tracks, self.Tracks_cb,queue_size=1)
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


    def Marker_delete(self):
        # delete_marker_array = MarkerArray()
        # for marker in self.marker_array.markers:
        #     marker.action = Marker.DELETE  # 设置删除操作
        #     delete_marker_array.markers.append(marker)
        # self.marker_pub.publish(delete_marker_array)
        # self.marker_array = MarkerArray() 
        self.marker_array = MarkerArray()
        self.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 0))
        self.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 1))
        self.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 300))
        self.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 301))
        self.marker_pub.publish(self.marker_array)
        self.marker_array = MarkerArray()



        # for id in range(11):
        # # 创建一个Marker消息，用于删除
        #     marker = Marker()
        #     marker.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        #     marker.ns = "numbers"
        #     marker.id = id
        #     marker.action = Marker.DELETE  # 设置删除操作

        # # 发布删除Marker消息
        # self.number_marker_pub.publish(marker)


        # marker = Marker()
        # # 填充Marker消息的必要字段
        # marker.header = Header(stamp=rospy.Time.now(), frame_id='map')
        # marker.ns = "numbers"
        # marker.type = Marker.TEXT_VIEW_FACING
        # marker.action = Marker.DELETE
        # marker.text = "haha"  # 可以将此数字动态更新
        # marker.pose.position.x = 100
        # marker.pose.position.y = 100
        # marker.pose.position.z = 100
        # marker.scale.z = 1.0
        # marker.color.r = 1.0
        # marker.color.g = 1.0
        # marker.color.b = 1.0
        # marker.color.a = 0.0
        # for id in range(11):
        #     marker.id = id
        #     self.number_marker_pub.publish(marker)



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




    def Tracks_cb(self, msg:tracks):
            
        for i, id in enumerate(msg.track_id_list):
            if id  == 0:
                # pose = [msg.track_pose_list.poses[i].position.x,msg.track_pose_list.poses[i].position.y] 
                pose = [-15.04079437255859 + 0.1*np.random.random(), 1.3136539459228516 + 0.1*np.random.random()]
                self.baoan_pose = pose
                self.marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 120))
                self.number_publish(pose[0], pose[1], 2.0,1.0, 1)
            # if id  == 34 and self. time < (1726930270.388218) :
            #     pose = [msg.track_pose_list.poses[i].position.x,msg.track_pose_list.poses[i].position.y] 
            #     self.nvsheng = pose
            #     self.marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 160))
            #     self.number_publish(pose[0], pose[1], 2.0,1.0, 6)





        if self.time < ellipse_1[1]:
            rospy.loginfo("if !!!!!!!!!!!!!!!!!!!!")

            group_pose = []
            if self.time < 1726930206.978113222:
                for i, id in enumerate(msg.track_id_list):
                    if id in id_list_predefine:
                        group_pose.append([msg.track_pose_list.poses[i].position.x,msg.track_pose_list.poses[i].position.y] )

            for id, pose in enumerate(self.last_group):
                self.number_publish(pose[0], pose[1], 2.0, 0.0, 3-id)
                rospy.loginfo(f"delete{3 - id}")

            for id, pose in enumerate(group_pose):
                self.marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 3 - id))
                self.number_publish(pose[0], pose[1], 2.0,1.0, 3 - id)
                rospy.loginfo(f"print{3 - id}")

            self.last_group =group_pose
                

            if len(group_pose) >= 2:
                if self.time > ellipse_1[0]:
                    ellipses = get_ellipses(np.array(group_pose),0.2)
                    for i, ellipse in enumerate(ellipses):
                        self.marker_array.markers.append(create_ellipse_marker(ellipse[0],ellipse[1],ellipse[2],ellipse[3],ellipse[4],marker_id = 300+i))
                    self.marker_pub.publish(self.marker_array)
        else:
            rospy.loginfo("else!!!!!!!!!!!!!!!!!!!!")
            group_pose = []
            group_id = -1
            for i, id in enumerate(msg.track_id_list):
                if id == 24:
                    group_id = 1
                    group_pose.append([poses[0][0] +  0.08 *np.random.random(),poses[0][1] +  0.08 *np.random.random()])
                if id == 25:
                    group_id = 1
                    group_pose.append([poses[1][0] +  0.08 *np.random.random(),poses[1][1] +  0.08 *np.random.random()])

                if id == 36:
                    group_id = 2
                    group_pose.append([poses[2][0] +  0.08 *np.random.random(),poses[2][1] +  0.08 *np.random.random()])
                if id == 37:
                    group_id = 2
                    group_pose.append([poses[3][0] +  0.08 *np.random.random(),poses[3][1] +  0.08 *np.random.random()])

                if id == 52:
                    group_id = 3
                    group_pose.append([poses[5][0] +  0.08 *np.random.random(),poses[5][1] +  0.08 *np.random.random()])

                if id == 59:
                    group_id = 3
                    group_pose.append([poses[4][0] +  0.08 *np.random.random(),poses[4][1] +  0.08 *np.random.random()])




            for id, pose in enumerate(self.last_group):
                if group_id == 1:
                    self.number_publish(pose[0], pose[1], 2.0, 0.0, id+4)
                if group_id == 2:
                    self.number_publish(pose[0], pose[1], 2.0, 0.0,id+7)
                if group_id == 3:
                    self.number_publish(pose[0], pose[1], 2.0, 0.0 ,id+9)

            for id, pose in enumerate(group_pose):
                self.marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, id))
                if group_id == 1:
                    self.number_publish(pose[0], pose[1], 2.0, 1.0, id+4)
                if group_id == 2:
                    self.number_publish(pose[0], pose[1], 2.0, 1.0,id+7)
                if group_id == 3:
                    self.number_publish(pose[0], pose[1], 2.0, 1.0 ,id+9)

            self.last_group =group_pose



            if len(group_pose) >= 2:
                if self.time > ellipse_starts[group_id]:
                    rospy.loginfo(f"scene{group_id+1} starts")

                    ellipses = get_ellipses(np.array(group_pose),0.2)
                    for i, ellipse in enumerate(ellipses):
                        self.marker_array.markers.append(create_ellipse_marker(ellipse[0],ellipse[1],ellipse[2],ellipse[3],ellipse[4],marker_id = 300+i))
            self.marker_pub.publish(self.marker_array)



        for end_time in ellipse_ends:
            # rospy.loginfo("end!!!!!!!!!!!!!!!!!!!!")
            if self.time > end_time + 0.8  and self.time < end_time + 1.5 :
                self.Marker_delete()

        

        if self.time > 1726930210 and self.time <  (1726930210 + 2):
            # rospy.loginfo("delete!!!!!!!!!!!!!!")
            self.Marker_delete()


        if self.time > 1726930204 and self.time <  (1726930204 + 1):
            # rospy.loginfo("delete!!!!!!!!!!!!!!")
            # self.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 120))
            # self.marker_pub.publish(self.marker_array)
            # self.number_publish(self.baoan_pose[0], self.baoan_pose[1], 2.0,0.0, 1)
            pass










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
    cnt = 0
    while not rospy.is_shutdown():
        cnt = cnt + 1
        try:

            for end_time in ellipse_ends:
                # rospy.loginfo("end!!!!!!!!!!!!!!!!!!!!")
                if rvizShower.time > end_time + 0.8  and rvizShower.time < end_time + 1.5 :
                    rvizShower.Marker_delete()

            # if rvizShower.time > 1726930270.388218 and  (rvizShower.time <  (1726930270.388218 + 3)):
            #     rospy.loginfo("girl delete!!!!!!!!!!!!!!")
            #     rospy.loginfo("girl delete!!!!!!!!!!!!!!")
            #     rospy.loginfo("girl delete!!!!!!!!!!!!!!")
            #     rospy.loginfo("girl delete!!!!!!!!!!!!!!")
            #     rvizShower.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 160))
            #     rvizShower.marker_pub.publish(rvizShower.marker_array)
            #     rvizShower.number_publish(rvizShower.nvsheng[0], rvizShower.nvsheng[1], 2.0,0.0, 6)

            if cnt >= 2:
                cnt = 0
                ha_begin_time = fxxk_time[0]
                ha_end_time = fxxk_time[-1]
                current_t = rvizShower.time
                if  current_t >= ha_begin_time and current_t <= ha_end_time:
                    min_time = 2000
                    min_index = 0
                    for k, time in enumerate(fxxk_time):
                        if math.fabs(current_t - time) < min_time:
                            min_time = math.fabs(current_t - time)
                            min_index = k
                    
                    pose = fxxk_pose[min_index]
                    x = pose[0]
                    x = x + 0.08*np.random.random()
                    y = pose[1]
                    y = y + 0.08*np.random.random()
                    pose = [x, y]

                    rvizShower.nvsheng = pose
                    rvizShower.marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 160))
                    rvizShower.number_publish(pose[0], pose[1], 2.0,1.0, 6)

                if current_t >= ha_end_time:
                    rospy.loginfo("girl delete!!!!!!!!!!!!!!")
                    rospy.loginfo("girl delete!!!!!!!!!!!!!!")
                    rospy.loginfo("girl delete!!!!!!!!!!!!!!")
                    rospy.loginfo("girl delete!!!!!!!!!!!!!!")
                    rvizShower.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 160))
                    rvizShower.marker_pub.publish(rvizShower.marker_array)
                    rvizShower.number_publish(rvizShower.nvsheng[0], rvizShower.nvsheng[1], 2.0,0.0, 6)

                if current_t >= 1726930213.288665 and current_t <= 1726930213.288665 + 0.8:
                    rvizShower.marker_array = MarkerArray()
                    rvizShower.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 0))
                    rvizShower.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 1))
                    rvizShower.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 300))
                    rvizShower.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 301))
                    rvizShower.marker_pub.publish(rvizShower.marker_array)
                    rvizShower.marker_array = MarkerArray()

                if current_t >= 1726930202.003986 and current_t <= 1726930202.003986 + 1:
                    rvizShower.marker_array = MarkerArray()
                    rvizShower.marker_array.markers.append(create_cylinder(-6.8, 1.5, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 0))
                    rvizShower.number_publish(-6.8, 1.5, 2.0, 1.0, 4)
                    rvizShower.marker_pub.publish(rvizShower.marker_array)
                    rvizShower.marker_array = MarkerArray()

            rvizShower.GetRobotPose()

        except Exception as e:
            rospy.logwarn(e)    
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
