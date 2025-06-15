import numpy as np
from trajectory_generator import TrajectoryGenerator



# # local trajectory generation test
global_path = np.array([[0.0, 0.2], [0.5, 0.2], [0.5, 0.8], [1.0, 0.8]])
pos = np.array([0.2, 0.2])
# single and repeated waypoints, enpoint test
traj_generator_4 = TrajectoryGenerator(horizon=10)
path_4 = traj_generator_4.generate_trajectory(pos, global_path)
print(path_4)

import matplotlib.pyplot as plt
plt.plot(global_path[:, 0], global_path[:, 1], 'ro-')
plt.plot(path_4[:, 0], path_4[:, 1], 'bo-')
plt.show()
