import numpy as np
from numba import float64, int64
from numba.experimental import jitclass

spec = [('p', float64[:]), ('v', float64[:]), ('yaw', float64),
        ('ba', float64[:]), ('bw', float64),
        ('Q', float64[:,:]), ('P', float64[:,:]), ('yawc', float64),
        ('H_gnss', float64[:,:]), ('H_compass', float64[:,:]),]

@jitclass(spec)
class imu_ekf_2d(object):
    def __init__(self, p0, v0, yaw0, ba0, bw0, P0, Q, yawc):
        self.p = p0
        self.v = v0
        self.yaw = yaw0
        self.ba = ba0
        self.bw = bw0
        self.Q = Q
        self.P = P0
        self.yawc = yawc

        ############# EDIT HERE
        self.H_gnss = np.zeros((2,8))
        self.H_gnss[:,:2] = np.eye(2)
        self.H_compass = np.zeros((1,8))
        self.H_compass[0,4] = 1.

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    def _yaw_to_rot(self, y):
        return np.array([[np.cos(y), -np.sin(y)],
                         [np.sin(y), np.cos(y)]])
    def _yaw_to_rot_dot(self, y):
        return np.array([[-np.sin(y), -np.cos(y)],
                         [np.cos(y), -np.sin(y)]])
    def get_state(self):
        return self.p, self.v, self.yaw, self.ba, self.bw
    def get_cov_sys(self):
        return self.P
    def predict(self, dt, a, w):
        C = self._yaw_to_rot(self.yaw - self.yawc)
        C_dot = self._yaw_to_rot_dot(self.yaw - self.yawc)

        self.p = self.p + dt*self.v + dt**2/2 * C@(a - self.ba)
        self.v = self.v + dt* C@(a - self.ba)
        self.yaw = self.yaw + dt*(w - self.bw)
        self.ba = self.ba
        self.bw = self.bw

        F = np.eye(8)
        F[:2, 2:4] = np.eye(2) * dt
        F[:2, 4] = dt**2/2 * C_dot@(a - self.ba)
        F[:2, 5:7] = - dt**2/2 * C
        F[2:4, 4] = dt*C_dot@(a - self.ba)
        F[2:4, 5:7] = -dt * C
        F[4, 7] = -dt

        L = np.zeros((8,6))
        L[:2,:2] = dt**2/2 * C
        L[:2,3:5] = - dt**2/2 * C
        L[2:4,:2] = dt*C
        L[2:4,3:5] = - dt*C
        L[4,2] = dt
        L[4,5] = -dt
        L[5:,3:] = np.eye(3)

        Q = np.copy(self.Q)
        Q[:3,:3] = Q[:3,:3] * dt**2
        Q[3:,3:] = Q[3:,3:] * dt

        self.P = F @ self.P @ F.T + L @ Q @ L.T

    def _kalman_gain(self, H, J):
        return self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + J)

    def _correct_state_from_inno(self, dx):
        self.p = self.p + dx[:2]
        self.v = self.v + dx[2:4]
        self.yaw = self._wrap_angle(self.yaw + dx[4])
        self.ba = self.ba + dx[5:7]
        self.bw = self.bw + dx[7]

    def _correct_cov_sys(self, K, H, J):
        self.P = (np.eye(8) - K @ H) @ self.P @ (np.eye(8) - K @ H).T + K @ J @ K.T

    def correct_compass(self, y, J):
        K = self._kalman_gain(self.H_compass, J)
        inno = self._wrap_angle(np.array([y - self.yaw]))
        dx = np.dot(K, inno)

        self._correct_state_from_inno(dx)
        self._correct_cov_sys(K, self.H_compass, J)

    def correct_gnss_2d(self, g2d, J):
        K = self._kalman_gain(self.H_gnss, J)
        inno = g2d - self.p
        dx = np.dot(K, inno)

        self._correct_state_from_inno(dx)
        self._correct_cov_sys(K, self.H_gnss, J)

print("Please Wait ...")
print("The imu_ekf_2d class is being compiled ...")

# COMPILING
yawc_imu = np.pi/2

var_a = 0.5
var_w = 0.1
var_ba = 0.5
var_bw = 0.001

Q = np.zeros((6,6))
Q[:2,:2] = np.eye(2) * var_a
Q[2,2] = var_w
Q[3:5, 3:5] = np.eye(2) * var_ba
Q[5, 5] = var_bw

J_gnss = np.eye(2) * 4.
J_compass = np.array([[0.075]])

ekf = imu_ekf_2d(np.zeros(2), np.zeros(2), 0.5, np.zeros(2), 0.5, np.zeros((8,8)), Q, yawc_imu)

_ = ekf._wrap_angle(0.1); _ = ekf._wrap_angle(np.array([0.1]))
_ = ekf._yaw_to_rot(0.5)
_ = ekf._yaw_to_rot_dot(0.5)
_ = ekf.get_state()
_ = ekf.get_cov_sys()
_ = ekf.predict(0.1, np.zeros(2), 0.5)
# K = ekf._kalman_gain(ekf.H_gnss, J_gnss); _ = ekf._kalman_gain(ekf.H_compass, J_compass)
# _ = ekf._correct_state_from_inno(np.zeros(8))
# _ = ekf._correct_cov_sys(K, ekf.H_gnss, J_gnss)
_ = ekf.correct_compass(0.5, J_compass)
_ = ekf.correct_gnss_2d(np.zeros(2), J_gnss)

print("The imu_ekf_2d class has been compiled !")
