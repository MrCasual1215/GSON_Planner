import copy
import math
import random
import time
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
import numpy as np
import cv2



class Informed_RRTstar:

    def __init__(self, costmap, resolution,
                 expandDis=1.0, goalSampleRate=10, maxIter=200, reconnect=False):

        self.start = None
        self.goal = None
        self.costmap = costmap
        self.width = self.costmap.shape[1]
        self.height = self.costmap.shape[0]
        self.resolution = resolution
        self.expand_dis = expandDis
        self.goal_sample_rate = goalSampleRate
        self.max_iter = maxIter
        self.reconnect = reconnect
        self.node_list = None
        


    def informed_rrt_star_planning(self, start, goal):
        start_time = time.time()
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.node_list = [self.start]

        # test_map = self.costmap.copy()
        # cv2.circle(test_map,(int(start[1]/self.resolution),int(start[0]/self.resolution)),2,128)
        # cv2.circle(test_map,(int(goal[1]/self.resolution),int(goal[0]/self.resolution)),2,128)

        # cv2.imwrite("/home/orin/arena_ws/src/ourplanner/map/middle_planner/test.png",test_map)


        if self._check(start) == False or  self._check(goal) == False:
            print("INVALID!!")
            return
        
        

        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        cBest = float('inf')
        path = None

        # Computing the sampling space
        cMin = math.sqrt(pow(self.start.x - self.goal.x, 2)
                         + pow(self.start.y - self.goal.y, 2))
        xCenter = np.array([[(self.start.x + self.goal.x) / 2.0],
                            [(self.start.y + self.goal.y) / 2.0], [0]])
        a1 = np.array([[(self.goal.x - self.start.x) / cMin],
                       [(self.goal.y - self.start.y) / cMin], [0]])

        e_theta = math.atan2(a1[1], a1[0])


        C = np.array([[math.cos(e_theta), -math.sin(e_theta), 0],
                      [math.sin(e_theta), math.cos(e_theta),  0],
                      [0,                 0,                  1]])

        i = -1
        FIND_PATH = False
        while i < self.max_iter or not FIND_PATH  :
            i = i + 1
            # Sample space is defined by cBest
            # cMin is the minimum distance between the start point and the goal
            # xCenter is the midpoint between the start and the goal
            # cBest changes when a new path is found

            rnd = self.informed_sample(cBest, cMin, xCenter, C)
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearestNode = self.node_list[n_ind]

            # steer
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.get_new_node(theta, n_ind, nearestNode)

            noCollision = self.check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            if noCollision:
                nearInds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearInds)

                self.node_list.append(newNode)
                self.rewire(newNode, nearInds)

                if self.is_near_goal(newNode):
                    if self.check_segment_collision(newNode.x, newNode.y,
                                                    self.goal.x, self.goal.y):
                        lastIndex = len(self.node_list) - 1
                        tempPath = self.get_final_course(lastIndex)
                        tempPathLen = self.get_path_len(tempPath)
                        if tempPathLen < cBest:
                            path = tempPath
                            cBest = tempPathLen
                            print("current path length: {}, It costs {} s".format(tempPathLen, time.time()-start_time))
                            FIND_PATH = True

        if self.reconnect:
            path = self.reconnect_path(path)
        path = self.reverse_path(path)
        return path



    def sample(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(0, self.height*self.resolution), random.uniform(0, self.width*self.resolution)]
        else:  # goal point sampling
            rnd = [self.goal.x, self.goal.y]
        return rnd
    
    def choose_parent(self, newNode, nearInds):
        if len(nearInds) == 0:
            return newNode

        dList = []
        for i in nearInds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.node_list[i], theta, d):
                dList.append(self.node_list[i].cost + d)
            else:
                dList.append(float('inf'))

        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("min cost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2
                  for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    def informed_sample(self, cMax, cMin, xCenter, C):
        if cMax < float('inf'):
            r = [cMax / 2.0,
                 math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
                 math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
            L = np.diag(r)
            xBall = self.sample_unit_ball()
            rnd = np.dot(np.dot(C, L), xBall) + xCenter
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sample()

        return rnd
    
    def _check(self,point):
        return (self.costmap[int(point[0]/self.resolution)][int(point[1]/self.resolution)] > 128)
    
    @staticmethod
    def reverse_path(path):
        n = len(path)
        path_r = []
        for i in range(n):
            path_r.append([path[n-i -1][0],path[n-i -1][1]])
        path_r = np.array(path_r)
        return path_r

    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    @staticmethod
    def line_cost(node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        dList = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)

        newNode.cost += self.expand_dis
        newNode.parent = n_ind
        return newNode

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    def rewire(self, newNode, nearInds):
        n_node = len(self.node_list)
        for i in nearInds:
            nearNode = self.node_list[i]

            d = math.sqrt((nearNode.x - newNode.x) ** 2
                          + (nearNode.y - newNode.y) ** 2)

            s_cost = newNode.cost + d

            if nearNode.cost > s_cost:
                theta = math.atan2(newNode.y - nearNode.y,
                                   newNode.x - nearNode.x)
                if self.check_collision(nearNode, theta, d):
                    nearNode.parent = n_node - 1
                    nearNode.cost = s_cost



    def check_segment_collision(self,  x1, y1, x2, y2):       
        point1 = (int(y1/self.resolution), int(x1/self.resolution))
        point2 = (int(y2/self.resolution), int(x2/self.resolution))

        line = cv2.line(np.zeros_like(self.costmap), point1, point2, color=255, thickness=1)


        for point in zip(*line.nonzero()):
            if self.costmap[point] < 127:
                return False  # collision
        return True

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        path = [[self.goal.x, self.goal.y]]
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path
    
    def reconnect_path(self,path):
        current_index = 0
        n = len(path)
        elimination = []

        #reconnect
        while current_index < n - 1:
                i = n - 1 
                while i > current_index + 1:
                    if self.check_segment_collision(path[i][0],path[i][1],path[current_index][0],path[current_index][1]) ==  True: # no collision
                        for j in range(current_index + 1,i):
                            elimination.append(j)
                        current_index = i
                        break
                    i = i - 1
                
                # update current_index
                current_index = i

        # results
        reconnect_path = np.delete(path,elimination,axis=0)
        return reconnect_path
            




class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def xy2index(x):
    return int(x*20)


def main():
    show_animation = True

    # get costmap
    costmap = cv2.imread('/home/orin/arena_ws/src/ourplanner/map/middle_planner/costmap.png', cv2.IMREAD_GRAYSCALE)

    # set parameter
    t1 = time.time()
    rrt = Informed_RRTstar(costmap, resolution=0.05, expandDis=1.0, goalSampleRate= 50, maxIter=80, reconnect=False)
    start = [22, 17]
    goal = [14, 10]
    path = rrt.informed_rrt_star_planning(start, goal)
    print(path)
    print(time.time() - t1)
    print("Done!!")

    if show_animation :
        for i in range(len(path) - 1):
            cv2.line(costmap, (xy2index(path[i][1]),xy2index(path[i][0])), (xy2index(path[i+1][1]),xy2index(path[i+1][0])), color=0, thickness=1)
        cv2.imwrite("/home/orin/arena_ws/src/ourplanner/map/middle_planner/test.png",costmap)



if __name__ == '__main__':
    main()