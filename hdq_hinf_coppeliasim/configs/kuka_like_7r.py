import numpy as np

deg = np.pi / 180.0

# CoppeliaSim DH Extractor output:
# [a, alpha, d, theta_offset, joint_type]
# a = r
# joint_type: 0 = revolute
DH_TABLE = np.array([
    [0.0, -90.0 * deg,  0.2000,  180.0 * deg, 0],
    [0.0, -90.0 * deg,  0.0000,   -0.0 * deg, 0],
    [0.0, -90.0 * deg,  0.2000,  180.0 * deg, 0],
    [0.0,  90.0 * deg, -0.0000, -180.0 * deg, 0],
    [0.0,  90.0 * deg,  0.1900,   -0.0 * deg, 0],
    [0.0, -90.0 * deg, -0.0000,    0.0 * deg, 0],
    [0.0,   0.0 * deg,  0.0000,    0.0 * deg, 0],
], dtype=float)

