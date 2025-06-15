import math
import os
import time
import cv2
import sys
from planner_utils.floyd_Bspline import Floyd_Bspline

class AStarPlanner:
    def __init__(self, costmap, resolution):
        self.costmap  = costmap
        self.width_ = costmap.shape[1]
        self.height_ = costmap.shape[0]
        self.resolution_ = resolution
        self.obstacle_map = [[True for _ in range(self.width_)] for _ in range(self.height_)]
        self.motion = self.get_motion_model()
        # obstacle map
        for y in range(self.width_):
            for x in range(self.height_):
                if self.costmap[x][y] < 100:
                    self.obstacle_map[x][y] = False 

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index


    def xy2index(self, position):
        return int((position)/self.resolution_)

    def index2xy(self,index):
        return index * self.resolution_ 

    def index2cnt(self, node:Node):
        return node.y * self.height_ + node.x
    
    def collision_check(self, node):
        if self.obstacle_map[node.x][node.y] == False: # collided
            return False
        
        return True
    
    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.xy2index(sx),self.xy2index(sy), 0.0, -1)
        goal_node = self.Node(self.xy2index(gx),self.xy2index(gy), 0.0, -1)
        # check valid
        if ( start_node.x >= self.height_ or start_node.x < 0 ) or ( start_node.y >= self.width_ or start_node.y < 0):
            print("Invalid start node!")
            quit()
        if ( goal_node.x >= self.height_ or goal_node.x < 0 ) or ( goal_node.y >= self.width_ or goal_node.y < 0):
            print("Invalid goal!")
            quit()
        if self.costmap[goal_node.x][goal_node.y] == False:
            print("Can't find astar path! The goal is not valid")
            quit()

        open_set, closed_set = dict(), dict()
        open_set[self.index2cnt(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            
            # find point with minimum cost 
            c_id = min(open_set,key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,open_set[o]))
            current = open_set[c_id]

            # reach goal
            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.index2cnt(node)

                if self.collision_check(node) == False:
                    continue   # if node is not safe,do nothing

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        rx.reverse()
        ry.reverse()

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.index2xy(goal_node.x)], [
            self.index2xy(goal_node.y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.index2xy(n.x))
            ry.append(self.index2xy(n.y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.001  # weight of heuristic
        dx = math.fabs(n1.x - n2.x)
        dy = math.fabs(n1.y - n2.y)
        h = (dx + dy) + (math.sqrt(2) - 2)*min(dx,dy)
        h = h * w
        # h = w * math.hypot(n1.x - n2.x, n1.y - n2.y)

        return h

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


