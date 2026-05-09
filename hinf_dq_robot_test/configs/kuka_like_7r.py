import numpy as np

# [a, alpha, d, theta_offset, joint_type]
# joint_type: 0 revolute, 1 prismatic
DH_TABLE = np.array([
    [0.0,  np.pi / 2, 0.34, 0.0, 0],
    [0.0, -np.pi / 2, 0.00, 0.0, 0],
    [0.0, -np.pi / 2, 0.40, 0.0, 0],
    [0.0,  np.pi / 2, 0.00, 0.0, 0],
    [0.0,  np.pi / 2, 0.40, 0.0, 0],
    [0.0, -np.pi / 2, 0.00, 0.0, 0],
    [0.0,  0.0,       0.126, 0.0, 0],
], dtype=float)