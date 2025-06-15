"""
Batch Informed Trees (BIT*)
@author: huiming zhou
"""

import math
import time
import cv2
import random
import numpy as np




class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Tree:
    def __init__(self, x_start, x_goal):
        self.x_start = x_start
        self.goal = x_goal

        self.r = 4.0
        self.V = set()
        self.E = set()
        self.QE = set()
        self.QV = set()

        self.V_old = set()


class BITStar:
    def __init__(self, eta, iter_max, costmap, resolution, ellipses_group):

        self.eta = eta
        self.iter_max = iter_max
        self.costmap = costmap
        self.resolution = resolution
        self.ellipses_group = ellipses_group
        self.safe_dis = 0.8

        self.x_range = [0,costmap.shape[0]*resolution]
        self.y_range = [0,costmap.shape[1]*resolution]


    def init(self):
        self.Tree.V.add(self.x_start)
        self.X_sample.add(self.x_goal)

        self.g_T[self.x_start] = 0.0
        self.g_T[self.x_goal] = np.inf

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])

        return theta, cMin, xCenter, C

    def planning(self, x_start, x_goal):

        x_start = self._check(x_start)
        x_goal = self._check(x_goal)
        self.x_start = Node(x_start[0], x_start[1])
        self.x_goal = Node(x_goal[0], x_goal[1])


        if not self.is_collision(self.x_start, self.x_goal):
            
            dis = math.sqrt((self.x_start.x - self.x_goal.x)**2+(self.x_start.y - self.x_goal.y)**2)
            delta_x = self.x_goal.x - self.x_start.x
            delta_y = self.x_goal.y - self.x_start.y
            delta_dis = 0.5
            step = max(int(dis/delta_dis),1)

            path_x, path_y = [], []
            for i in range(step):
                x = (self.x_start.x + delta_x * i / float(step))
                y = (self.x_start.y + delta_y * i / float(step))
                path_x.append(x)
                path_y.append(y)
            path_x.append(self.x_goal.x)
            path_y.append(self.x_goal.y)
            return path_x, path_y



        self.Tree = Tree(self.x_start, self.x_goal)
        self.X_sample = set()
        self.g_T = dict()

        theta, cMin, xCenter, C = self.init()

        for k in range(self.iter_max):
            if not self.Tree.QE and not self.Tree.QV:
                if k == 0:
                    m = 200
                else:
                    m = 100


                self.Prune(self.g_T[self.x_goal])
                self.X_sample.update(self.Sample(m, self.g_T[self.x_goal], cMin, xCenter, C))
                self.Tree.V_old = {v for v in self.Tree.V}
                self.Tree.QV = {v for v in self.Tree.V}
                # self.Tree.r = self.radius(len(self.Tree.V) + len(self.X_sample))

            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                self.ExpandVertex(self.BestInVertexQueue())

            vm, xm = self.BestInEdgeQueue()
            self.Tree.QE.remove((vm, xm))

            if self.g_T[vm] + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.g_T[self.x_goal]:
                actual_cost = self.cost(vm, xm)
                if self.g_estimated(vm) + actual_cost + self.h_estimated(xm) < self.g_T[self.x_goal]:
                    if self.g_T[vm] + actual_cost < self.g_T[xm]:
                        if xm in self.Tree.V:
                            # remove edges
                            edge_delete = set()
                            for v, x in self.Tree.E:
                                if x == xm:
                                    edge_delete.add((v, x))

                            for edge in edge_delete:
                                self.Tree.E.remove(edge)
                        else:
                            self.X_sample.remove(xm)
                            self.Tree.V.add(xm)
                            self.Tree.QV.add(xm)

                        self.g_T[xm] = self.g_T[vm] + actual_cost
                        self.Tree.E.add((vm, xm))
                        xm.parent = vm

                        set_delete = set()
                        for v, x in self.Tree.QE:
                            if x == xm and self.g_T[v] + self.calc_dist(v, xm) >= self.g_T[xm]:
                                set_delete.add((v, x))

                        for edge in set_delete:
                            self.Tree.QE.remove(edge)
            else:
                self.Tree.QE = set()
                self.Tree.QV = set()

        path_x, path_y = self.ExtractPath()
        path_x.reverse()
        path_y.reverse()
        return path_x, path_y

    def ExtractPath(self):
        node = self.x_goal
        path_x, path_y = [node.x], [node.y]

        while node.parent:
            node = node.parent
            path_x.append(node.x)
            path_y.append(node.y)

        return path_x, path_y

    def Prune(self, cBest):
        self.X_sample = {x for x in self.X_sample if self.f_estimated(x) < cBest}
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest}
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}
        self.X_sample.update({v for v in self.Tree.V if self.g_T[v] == np.inf})
        self.Tree.V = {v for v in self.Tree.V if self.g_T[v] < np.inf}

    def is_collision(self, start, end):

        x1 = int(start.x/self.resolution)
        x2 = int(end.x/self.resolution)
        y1 = int(start.y/self.resolution)
        y2 = int(end.y/self.resolution)


        dis = math.sqrt((x2-x1)**2+(y1-y2)**2)
        delta_x = x2 - x1
        delta_y = y2 - y1
        step = max(int(dis),1)
        for i in range(step):
            x = int((x1 + delta_x * i / float(step)))
            y = int((y1 + delta_y * i / float(step)))
            if self.costmap[x][y] < 127:
                return True
            # collision
        
        if self.costmap[x2][y2] < 127 :
            return True
        
        return False

    def in_obs(self, node):
        x = int(node.x/self.resolution)
        y = int(node.y/self.resolution)
        if self.costmap[x][y] < 127:
            return True
        
        return False
    
    def _check(self,point):



        x =  int(point[0]/self.resolution)
        y =  int(point[1]/self.resolution)

        if self.costmap[x][y] < 128: 
            VALID = False
            r = 0
            while not VALID:
                r = r + 1
                for x_,y_ in zip(range(max(0,x-r),min(self.costmap.shape[0],x+r)),range(max(0,y-r),min(self.costmap.shape[1],y+r))):
                    if self.costmap[x_][y_] > 128:
                        VALID = True
                        x = x_
                        y = y_
                        break
        
        return [x*self.resolution,y*self.resolution]
                
            
                    

                
        


        x = point[0]
        y = point[1]

        p0 = np.array([x,y])
    
        for ellipses in self.ellipses_group:
            x_center = 0
            y_center = 0

            for i  in range(len(ellipses)): 
                x_center = (i/float(i+1)) * x_center + (1.0/float(i+1)) * ellipses[i][0]
                y_center = (i/float(i+1)) * y_center + (1.0/float(i+1)) * ellipses[i][1]
            center = np.array([x_center,y_center])

            VALID  = False
            step = 0.2

            while not VALID:
                VALID  = True
                for ellipse in ellipses:
                    h, k = ellipse[0], ellipse[1]
                    a, b = ellipse[2] + self.safe_dis , ellipse[3] + self.safe_dis 
                    theta = ellipse[4]

                    x_rot = (x - h) * np.cos(theta) + (y - k) * np.sin(theta)
                    y_rot = (y - k) * np.cos(theta) - (x - h) * np.sin(theta)

                    if (x_rot / a)**2 + (y_rot / b)**2 <= 1:
                        VALID  = False
                if not VALID:
                    p = (p0 - center)/np.linalg.norm(p0 - center)
                    p0 = p0 + p * step
                    x = p0[0]
                    y = p0[1]

        return [x,y]
    
    

    def cost(self, start, end):
        if self.is_collision(start, end):
            return np.inf

        return self.calc_dist(start, end)

    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:
            return self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            return self.SampleFreeSpace(m)

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2 + 1e-5) / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2 + 1e-5) / 2.0]
        L = np.diag(r)

        ind = 0
        Sample = set()

        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter
            node = Node(x_rand[(0, 0)], x_rand[(1, 0)])
            in_obs = self.in_obs(node)
            in_x_range = self.x_range[0]  <= node.x <= self.x_range[1]
            in_y_range = self.y_range[0]  <= node.y <= self.y_range[1] 

            if not in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1

        return Sample

    def SampleFreeSpace(self, m):
        Sample = set()

        ind = 0
        while ind < m:
            node = Node(random.uniform(self.x_range[0] , self.x_range[1] ),
                        random.uniform(self.y_range[0] , self.y_range[1] ))
            if self.in_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample

    def radius(self, q):
        cBest = self.g_T[self.x_goal]
        lambda_X = len([1 for v in self.Tree.V if self.f_estimated(v) <= cBest])
        radius = 2 * self.eta * (1.5 * lambda_X / math.pi * math.log(q) / q) ** 0.5

        return radius

    def ExpandVertex(self, v):
        self.Tree.QV.remove(v)
        X_near = {x for x in self.X_sample if self.calc_dist(x, v) <= self.Tree.r}

        for x in X_near:
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.g_T[self.x_goal]:
                self.g_T[x] = np.inf
                self.Tree.QE.add((v, x))

        if v not in self.Tree.V_old:
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}

            for w in V_near:
                if (v, w) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.g_T[self.x_goal] and \
                        self.g_T[v] + self.calc_dist(v, w) < self.g_T[w]:
                    self.Tree.QE.add((v, w))
                    if w not in self.g_T:
                        self.g_T[w] = np.inf

    def BestVertexQueueValue(self):
        if not self.Tree.QV:
            return np.inf

        return min(self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV)

    def BestEdgeQueueValue(self):
        if not self.Tree.QE:
            return np.inf

        return min(self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE)

    def BestInVertexQueue(self):
        if not self.Tree.QV:
            print("QV is Empty!")
            return None

        v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}

        return min(v_value, key=v_value.get)

    def BestInEdgeQueue(self):
        if not self.Tree.QE:
            print("QE is Empty!")
            return None

        e_value = {(v, x): self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE}

        return min(e_value, key=e_value.get)

    @staticmethod
    def SampleUnitNBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)

    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)





def main():
    costmap = cv2.imread("/home/orin/arena_ws/src/ourplanner/map/test/costmap22.png",cv2.IMREAD_GRAYSCALE)
    x_start = (5, 5)  # Starting node
    x_goal = (25, 10)  # Goal node
    eta = 2
    iter_max = 300
    print("start!!!")
    bit = BITStar(eta, iter_max, costmap, 0.05, [])

    t1 = time.time()
    path_x, path_y = bit.planning(x_start, x_goal)
    print(time.time() - t1)
    print(path_x,path_y)

    show_map = costmap.copy()
    for i in range(len(path_x)-1):
        cv2.line(show_map,(int(path_y[i]*20),int(path_x[i]*20)), (int(path_y[i+1]*20),int(path_x[i+1]*20)), 128,2)
    cv2.imwrite("/home/orin/arena_ws/src/ourplanner/map/test/bit.png",show_map)
  


if __name__ == '__main__':
    main()