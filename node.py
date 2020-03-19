#!/usr/bin/env python

import numpy as np
from math import cos, sin, asin, atan2


def get_quaternion(r, p, y):
    """Convert given roll, pitch, yaw to quaternion."""
    c_1 = cos(r*0.5)
    s_1 = sin(r*0.5)
    c_2 = cos(p*0.5)
    s_2 = sin(p*0.5)
    c_3 = cos(y*0.5)
    s_3 = sin(y*0.5)

    w = c_1*c_2*c_3+s_1*s_2*s_3
    x = -c_1*s_2*s_3+c_2*c_3*s_1
    y = c_1*c_3*s_2+s_1*c_2*s_3
    z = c_1*c_2*s_3-s_1*c_3*s_2
    return x, y, z, w


def get_euler(x, y, z, w):
    """Convert given quaternion to roll, pitch, yaw. NOTE: Singularity for pitch equals 90 degrees"""
    r = atan2(2*(y*z + w*x), z**2-y**2-x**2+w**2)
    p = -asin(2*(x*z-w*y))
    y = atan2(2*(x*y+w*z), x**2+w**2-z**2-y**2)
    return r, p, y


class StateEstimator:

    def __init__(self, P, O, P_cov, O_cov):
        """"Take position P (x y z), orientation O (x y z w) and 3x3 Position covariance and
        Orientation covariance and make these the initial state (x y z roll pitch yaw) and covariance.
        Also defines tuning variables"""
        # Tuning variables
        self.sigma_u = np.diag([0.5, 0.01, 0.01, 0.01]) # (v omega_x omega_y omega_z)
        self.sigma_a = np.diag([5, 0.01, 0.01, 0.01])  # (a alpha_x alpha_y alpha_z)

        # Initialisation of matrices and vectors
        self.X_est = None
        self.P_est = None
        self.H = np.eye(6)

        # Initial state and covariance
        r, p, y = get_euler(O[0], O[1], O[2], O[3])
        self.X_prev = np.array([[P[0]], [P[1]], [P[2]], [r], [p], [y]])
        self.P_prev = np.diag([P_cov[0], P_cov[4], P_cov[8], O_cov[0], O_cov[4], O_cov[8]])

    def predict(self, v, omega, t):
        """Kalman filter prediction via wheel speed(m/s) and IMU (roll pitch yaw) rate measurements"""
        # Rear odometric model prediction
        self.X_est = self.X_prev + np.array([[v*t*cos(self.X_prev[5]+omega[2]*t/2)],
                                             [v*t*sin(self.X_prev[5]+omega[2]*t/2)],
                                             [v*t*sin(self.X_prev[4]+omega[1]*t/2)],
                                             [omega[0]*t],
                                             [omega[1]*t],
                                             [omega[2]*t]])

        # Jacobian w.r.t state variables
        F_x = np.array([[1, 0, 0, 0, 0, -t*v*sin(self.X_prev[5]+omega[2]*t/2)],
                        [0, 1, 0, 0, 0, t*v*cos(self.X_prev[5]+omega[2]*t/2)],
                        [0, 0, 1, 0, t*v*cos(self.X_prev[4]+omega[1]*t/2), 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]])

        # Jacobian w.r.t input variables
        F_u = np.array([[t*cos(self.X_prev[5]+omega[2]*t/2), 0, 0, -(t**2*v*sin(self.X_prev[5]+omega[2]*t/2))/2],
                        [t*sin(self.X_prev[5]+omega[2]*t/2), 0, 0, (t**2*v*cos(self.X_prev[5]+omega[2]*t/2))/2],
                        [t*sin(self.X_prev[4]+omega[1]*t/2), 0, (t**2*v*cos(self.X_prev[4]+omega[1]*t/2))/2, 0],
                        [0, t, 0, 0],
                        [0, 0, t, 0],
                        [0, 0, 0, t]])

        self.P_est = F_x.dot(self.P_prev).dot(F_x.T) + F_u.dot(self.sigma_u).dot(F_u.T)

    def update(self, z_P, z_O, R):
        """Kalman filter update via GNSS or LiDAR observation"""
        # Converting measurement to correct state
        r, p, y = get_euler(z_O[0], z_O[1], z_O[2], z_O[3])
        z = np.array([[z_P[0]], [z_P[1]], [z_P[2]], [r], [p], [y]])

        # Kalman update calculations
        S = self.H.dot(self.P_est).dot(self.H.T) + R
        K = self.P_est.dot(self.H.T).dot(np.linalg.inv(S))
        self.X_est = self.X_est + K.dot(z-self.X_est)
        self.P_est = (np.eye(6)-K.dot(self.H)).dot(self.P_est)

    def finish_loop(self):
        self.X_prev = self.X_est
        self.P_prev = self.P_est
