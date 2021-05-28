#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition



dT = 0.01

# Kalman Filter
kalman_filter_ = KalmanFilter(dim_x=18, dim_z=18)

# State transition matrix
f = np.array([
    [1., dT, 0.5*dT**2.],
    [0., 1.,         dT],
    [0., 0.,         1.]
])
kalman_filter_.F = scipy.linalg.block_diag(*[f]*6)

# Observation matrix
kalman_filter_.H = np.eye(18)

# Process noise
Q1 = Q_discrete_white_noise(dim=3, dt=dT, var=1e-2, block_size=3)
Q2 = Q_discrete_white_noise(dim=3, dt=dT, var=1e-2, block_size=3)
kalman_filter_.Q = np.block([[Q1,np.zeros((9,9))],
                            [np.zeros((9,9)),Q2]])
h = np.block([
    [-1., np.zeros((1,8)), 1., np.zeros((1,8))],
    [np.zeros((1,3)), -1., np.zeros((1,8)), 1., np.zeros((1,5))],
    [np.zeros((1,6)), -1., np.zeros((1,8)), 1., np.zeros((1,2))]
])

print(h.shape)

H = np.block([
    [-1., np.zeros((1,8)), 1., np.zeros((1,8))],
    [np.zeros((1,3)), -1., np.zeros((1,8)), 1., np.zeros((1,5))],
    [np.zeros((1,6)), -1., np.zeros((1,8)), 1., np.zeros((1,2))],
    [np.zeros((15,18))]
])
print(H)