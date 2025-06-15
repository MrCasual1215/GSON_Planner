import numpy as np


class TrajectoryGenerator:
    def __init__(self, horizon):
        self._global_path_index = 0
        self._num_waypoint = None
        self._reference_speed = 0.4
        self._num_horizon = horizon
        self._local_path_timestep = 0.1
        self._local_trajectory = None
        self._proj_dist_buffer = 0.05

    def generate_trajectory(self, pos, global_path):
        # TODO: move initialization of _num_waypoint and _global_path to constructor
        self._global_path_index = 0
        self._global_path = global_path
        self._num_waypoint = global_path.shape[0]
        # TODO: pass _global_path as a reference
        return self.generate_trajectory_internal(pos, self._global_path)

    def generate_trajectory_internal(self, pos, global_path):
        local_index = np.argmin(np.linalg.norm(global_path - pos, axis=1))
        trunc_path = np.vstack([global_path[local_index:, :], global_path[-1, :]])
        curv_vec = trunc_path[1:, :] - trunc_path[:-1, :]
        # print("curv_vec", curv_vec)
        curv_length = np.linalg.norm(curv_vec, axis=1)
        # print("curv_length", curv_length)

        if curv_length[0] == 0.0:
            curv_direct = np.zeros((2,))
        else:
            curv_direct = curv_vec[0, :] / curv_length[0]
        # print("curv_direct", curv_direct)
        proj_dist = np.dot(pos - trunc_path[0, :], curv_direct)


        # TODO: make the if statement optional
        if proj_dist <= 0.0:
            proj_dist = 0.0

        t_c = (proj_dist + self._proj_dist_buffer) / self._reference_speed
        t_s = t_c + self._local_path_timestep * np.linspace(0, self._num_horizon - 1, self._num_horizon)
        # print("t_s", t_s)

        curv_time = np.cumsum(np.hstack([0.0, curv_length / self._reference_speed]))
        # print("curv_time", curv_time)
        # np.cumsum使用方法：np.cumsum(a, axis=None, dtype=None, out=None)
        # a:输入数组
        # axis:沿着它累加值的轴
        # dtype:输出数组的类型
        # out:输出结果

        curv_time[-1] += (
            t_c + 2 * self._local_path_timestep * self._num_horizon + self._proj_dist_buffer / self._reference_speed
        )
        # print("curv_time", curv_time)


        # print("ts", t_s)
        path_idx = np.searchsorted(curv_time, t_s, side="right") - 1
        # print("path_idx", path_idx)
        # np.searchsorted使用方法：np.searchsorted(a, v, side='left', sorter=None)
        # a:输入数组
        # v:要查找的元素或数组
        # side:查找方向，当为left时，在左侧找到第一个符合条件的值，当为right时，在右侧找到第一个符合条件的值
        # sorter:排序数组

        # print("trunc_path", trunc_path)
        # print("curv_vec", curv_vec)

        path_human = np.vstack(
            [
                np.interp(t_s, curv_time, trunc_path[:, 0]),
                np.interp(t_s, curv_time, trunc_path[:, 1]),
            ]
        ).T

        # np.interp使用方法：np.interp(x, xp, fp, left=None, right=None, period=None)
        # x:要插值的x坐标
        # xp:已知点的x坐标
        # fp:已知点的y坐标
        # left:左侧的值
        # right:右侧的值
        # period:周期

        # path_human_head = np.arctan2(curv_vec[path_idx, 1], curv_vec[path_idx, 0]).reshape(self._num_horizon, 1)
        # path_dog = np.array([GuideDogDynamics.human2dog([path_human[i][0], path_human[i][1],path_human_head[i][0]]) for i in range(len(path_human)) ])

        # path_dog_head =[]
        # for point in path_dog:
        #     min_dist_index = np.argmin(np.linalg.norm(point - trunc_path, axis=1))
        #     theta = np.arctan2(curv_vec[min_dist_index][1],curv_vec[min_dist_index][0])
        #     path_dog_head.append(theta)

        # path_dog_head = np.array(path_dog_head).reshape(self._num_horizon, 1)
        # if len(path_dog_head) > 1:
        #     path_dog_head[-1] = path_dog_head[-2]
        
        # self._local_trajectory = np.hstack([path_human, path_dog, path_dog_head])
        self._local_trajectory = path_human
        return self._local_trajectory

