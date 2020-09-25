import numpy as np
from numba import float64, int64
from numba.experimental import jitclass

class ukf_imu(object):
    def __init__(self, p0, v0, q0, ba0, bw0, r_eff, P0, ...):
        self.p = p0
        self.v = v0
        self.q = q0
        self.ba = ba0
        self.bw = bw0
        self.br = 0.0
        self.P = P0
        self.N = self.P.shape[0]
        self.r_eff = r_eff

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _skew_symmetric(self, a, b, c):
        return np.array([[0, -c, b],
                         [c, 0, -a],
                         [-b, a, 0] ])

    def _qt_to_euler(self, q):
        # Rotation Sequence: yaw --> pitch --> roll
        r = np.arctan2(2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2 * (q[1]**2 + q[2]**2))
        p = np.arcsin(2 * (q[0]*q[2] - q[3]*q[1]))
        y = np.arctan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
        return np.array([r, p, y])

    def _qt_to_rot_mat(self, q):
        w, x, y, z = q
        ww = w*w
        xx = x*x
        yy = y*y
        zz = z*z
        wx = w*x
        wy = w*y
        wz = w*z
        xy = x*y
        xz = x*z
        yz = y*z

        r1 = 1 - 2*y**2 - 2*z**2
        r2 = 2*xy - 2*wz
        r3 = 2*xz + 2*wy
        r4 = 2*xy + 2*wz
        r5 = 1 - 2*x**2 - 2*z**2
        r6 = 2*yz - 2*wx
        r7 = 2*xz - 2*wy
        r8 = 2*yz + 2*wx
        r9 = 1 - 2*x**2 - 2*y**2

        return np.array([[r1, r2, r3],
                         [r4, r5, r6],
                         [r7, r8, r9]])

    def _qt_product(self, left, right):
        # Quaternion product or multiplication
        qv = np.array([left[1], left[2], left[3]])
        m = np.zeros([4, 4])
        m[0,1:] = -qv
        m[1:,0] = qv
        m[1:,1:] = self._skew_symmetric(qv[0], qv[1], qv[2])
        return np.array(np.dot(left[0] * np.eye(4) + m, right))

    def get_state(self):
        return self.p, self.v, self.q, self.ba, self.bw, self.br

    def get_cov_sys(self):
        return self.P

    def predict(self, dt, a, w):
        
