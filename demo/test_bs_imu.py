import numpy as np
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
from bs_imu import bs_imu


rpy = np.rad2deg([90,0,90])
quat = bs_imu.rpy_to_quat(rpy)
mat = bs_imu.rpy_to_matrix(rpy)

print(quat)
print(mat)