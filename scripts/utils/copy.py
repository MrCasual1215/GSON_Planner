












# starts = [1726930225.89,  1726930236.89,  1,1,1,1]
# ends = [1726930267.13, 1726930267.13, 1726930389,1726930389,1726930389,1726930389]
# stops = [1726930263.21, 1726930263.21, ]

# class Fxxxking_agent_manager:
#     def __init__(self, start_list, end_list, ids, poses, noise) -> None:
#         self.start_list = start_list
#         self.end_list = end_list
#         self.ids = ids
#         self.poses = poses
#         self.noise = noise
#         rospy.Subscriber("/odom", Odometry, self.Odom_cb,queue_size=1)
#         self.marker_pub = rospy.Publisher('/agents_marker_array', MarkerArray, queue_size=10)

    
#     def Update(self):
#         marker_array = MarkerArray()
#         group_poses = []
#         for i, pose in enumerate(self.poses):
#             if self.time >= self.start_list[i] and self.time <= self.end_list[i]:
#                 pose = [ pose[0] + self.noise*np.random.random(),  pose[1] + self.noise*np.random.random() ]
#                 group_poses.append(pose)
#                 marker_array.markers.append(create_cylinder(pose[0], pose[1], 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, self.ids[i]))
#             else:
#                 marker_array.markers.append(create_cylinder(100, 100, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, self.ids[i]))
#         if len(group_poses) >= 2 :
#             ellipses = get_ellipses(np.array(group_poses),0.2)
#             for i, ellipse in enumerate(ellipses):
#                 marker_array.markers.append(create_ellipse_marker(ellipse[0],ellipse[1],ellipse[2],ellipse[3],ellipse[4],marker_id = 300+i))
#             self.marker_array = MarkerArray()
#         self.marker_pub.publish(marker_array)

            
#     def Odom_cb(self, msg:Odometry):
#         self.time = msg.header.stamp.to_sec()



            # elif self.time > ellipse_1[0] + 3:
            #         self.marker_array = MarkerArray()
            #         self.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 0))
            #         self.marker_array.markers.append(create_cylinder(200, 200, 0.75, (0.38, 0.57, 0.62, 1.0), 0.4, 1.5, 1))
            #         self.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 300))
            #         self.marker_array.markers.append(create_ellipse_marker(100,100,1,1,1,marker_id = 301))
            #         self.marker_pub.publish(self.marker_array)
            #         self.marker_array = MarkerArray()