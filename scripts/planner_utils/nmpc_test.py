
import numpy as np
from trajectory_generator import TrajectoryGenerator
from optimizer import NmpcDcbfOptimizerParam, NmpcDbcfOptimizer

# # local trajectory generation test
global_path = np.array([[0.0, 0.2], [0.5, 0.2], [0.5, 0.8], [1.0, 0.8]])
pos = np.array([0.2, 0.2])
# single and repeated waypoints, enpoint test
traj_generator = TrajectoryGenerator(horizon=100)
local_path = traj_generator.generate_trajectory(pos, global_path)
print(local_path)

import matplotlib.pyplot as plt
plt.plot(global_path[:, 0], global_path[:, 1], 'ro-')
plt.plot(local_path[:, 0], local_path[:, 1], 'bo-')
plt.show()


state = np.array([[0.0, 0.0, 0.0]]).T

obstacles = np.array([[]])
# nmpc optimizer test
optimizer = NmpcDbcfOptimizer({},{},0.1, NmpcDcbfOptimizerParam())

for i in range(10):
    optimizer.setup(state, local_path, obstacles)
    u = optimizer.solve_nlp()   
    print("Control input:", u)

print(optimizer.setup_times)
print(optimizer.solver_times)
