import matplotlib.pyplot as plt
from scipy.interpolate import BSpline
import numpy as np
    
class Floyd_Bspline:
    def __init__(self,costmap,k,resolution):
         self.costmap_ = costmap
         self.bspline_k = k
         self.resolution = resolution
        
    def _isCollided(self,x1,y1,x2,y2):
        # check
        k = float((y2 - y1)/(x2 - x1 + 1e-5))
        b = y1 - k*x1

        for x in range(int(min(x1,x2)),int(max(x1,x2))):
                if self.costmap_[x][int(k*x + b)]< 100:
                    return True
        return False    
    
    def _xy2index(self,position):
         return int(position/self.resolution)
    
    def _find_min_dis(self,rx,ry,point):
        # find the index of minimum distance to a given point
        min_index = 0
        min_dis = (rx[0] - point[0])**2 + (ry[0] - point[1])**2
        for i in range(1,len(rx)):
             if ((rx[i] - point[0])**2 + (ry[i] - point[1])**2) < min_dis:
                  min_dis = (rx[i] - point[0])**2 + (ry[i] - point[1])**2
                  min_index = i
        return min_index
            
    def _interpolate_points(self,points, interval):

        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]

            # 计算两点之间的距离
            dist = np.linalg.norm(np.array(end_point) - np.array(start_point))

            # 计算需要插入的点数
            num_points_to_insert = int(np.floor(dist / interval))

            if  num_points_to_insert == 0 :
                continue

            # 生成新点
            for j in range(num_points_to_insert + 1):
                fraction = j / num_points_to_insert if num_points_to_insert > 0 else 0
                new_point = start_point + fraction * (np.array(end_point) - np.array(start_point))
                interpolated_points.append(tuple(new_point))

            # 避免重复添加终点
            if i < len(points) - 2:
                interpolated_points.pop()

        # 添加最后一个点
        # interpolated_points.append(points[-1])

        return interpolated_points

    def Trajectory_optimize(self,rx,ry,floyd,Bspline):
        n = len(rx)
        start = (self._xy2index(rx[0]),self._xy2index(ry[0]))
        goal = (self._xy2index(rx[n-1]),self._xy2index(ry[n-1]))

        # floyd
        if(floyd == True):
            current_index = 0
            elimination = []

            #reconnect
            while( current_index != n-1):
                i = n - 1 
                while( i > current_index + 1):
                    if(self._isCollided(self._xy2index(rx[current_index]),self._xy2index(ry[current_index]),self._xy2index(rx[i]),self._xy2index(ry[i])) == False):
                        for j in range(current_index + 1,i):
                            elimination.append(j)
                        current_index = i
                        break
                    i = i - 1
                
                # update current_index
                current_index = i

            # results
            f_x = np.delete(rx,elimination)
            f_y = np.delete(ry,elimination)
            

            points = np.array(list(zip(f_x,f_y)))
            interpolated_points = self._interpolate_points(points,0.3)
            n = len(interpolated_points)
            # print(interpolated_points)


        # Bspline
        if (Bspline == True):
            if(floyd==True):
                ctrl_points = interpolated_points
            else:
                if( n < 2 * self.bspline_k + 2):return f_x,f_y
                ctrl_points = np.array(list(zip(rx,ry)))

            k = self.bspline_k 
            t = np.concatenate(([0]*k, np.arange(n - k + 1), [n - k]*k))
            spl = BSpline(t, ctrl_points, k)
            x = np.linspace(0, n-k, 100)
            yy = spl(x)
                
            rx = list(yy[:,0])
            ry = list(yy[:,1])

        return rx,ry

