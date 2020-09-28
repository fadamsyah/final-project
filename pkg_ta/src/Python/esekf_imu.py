import numpy as np
from numba import float64, int64
from numba.experimental import jitclass

spec = [('p', float64[:]), ('v', float64[:]), ('q', float64[:]),
        ('ba', float64[:]), ('bw', float64[:]), ('br', float64), ('P', float64[:, :]),
        ('_g', float64[:]), ('_r_eff', float64), ('_D', float64),
        ('_Qi', float64[:, :]), ('_H_gnss', float64[:, :]),]

@jitclass(spec)
class ESEKF_IMU(object):
    def __init__(self, p0, v0, q0, ba0, bw0, r_eff, g, mag_dec, P0,
                 var_a, var_w, var_ba, var_bw, var_br):
        self.p = p0
        self.v = v0
        self.q = q0
        self.ba = ba0
        self.bw = bw0
        self.br = 0.0
        self.P = P0
        self._r_eff = r_eff
        self._g = g
        self._D = mag_dec

        self._Qi = np.zeros((13, 13))
        self._Qi[:3,:3] = np.eye(3) * var_a
        self._Qi[3:6,3:6] = np.eye(3) * var_w
        self._Qi[6:9,6:9] = np.eye(3) * var_ba
        self._Qi[9:12,9:12] = np.eye(3) * var_bw
        self._Qi[12,12] = var_br

        self._H_gnss = np.zeros((3, 17))
        self._H_gnss[:3,:3] = np.eye(3)

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _skew_symmetric(self, v):
        return np.array([[   0., -v[2],  v[1]],
                         [ v[2],    0., -v[0]],
                         [-v[1],  v[0],   0.]])

    def _qt_product(self, ql, qr):
        pw, px, py, pz = ql
        qw, qx, qy, qz = qr

        return np.array([pw*qw - px*qx - py*qy - pz*qz,
                         pw*qx + px*qw + py*qz - pz*qy,
                         pw*qy - px*qz + py*qw + pz*qx,
                         pw*qz + px*qy - py*qx + pz*qw])

    def euler_to_qt(self, rpy):
        r, p, y = rpy

        cr = np.cos(r/2)
        sr = np.sin(r/2)
        cp = np.cos(p/2)
        sp = np.sin(p/2)
        cy = np.cos(y/2)
        sy = np.sin(y/2)

        return np.array([cr*cp*cy + sr*sp*sy,
                         sr*cp*cy - cr*sp*sy,
                         cr*sp*cy + sr*cp*sy,
                         cr*cp*sy - sr*sp*cy])

    def qt_to_euler(self, q):
        w, x, y, z = q

        r = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x**2 + y**2))
        p = np.arcsin(2 * (w*y - z*x))
        y = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y**2 + z**2))

        return np.array([r, p, y])

    def _axis_angle_to_qt(self, v):
        norm = np.linalg.norm(v)
        q = np.zeros(4)

        q[0] = np.cos(norm/2)
        if norm < 10**(-10) :
            q[1:] = 0
        else :
            q[1:] = v / norm * np.sin(norm/2)

        return q

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

        return np.array([[ww + xx - yy - zz,       2*(xy - wz),       2*(xz + wy)],
                         [      2*(xy + wz), ww - xx + yy - zz,       2*(yz - wx)],
                         [      2*(xz - wy),       2*(yz + wx), ww - xx - yy + zz]])

    def get_state(self):
        return self.p, self.v, self.q, self.ba, self.bw, self.br

    def get_cov_sys(self):
        return self.P

    def predict(self, dt, a, w):
        # Rotation matrix from the quaternion
        C = self._qt_to_rot_mat(self.q)

        # Update the predicted nominal State
        self.p = self.p + dt * self.v + (dt**2)/2 * (np.dot(C, a - self.ba) + self._g)
        self.v = self.v + dt * (np.dot(C, a - self.ba) + self._g)
        self.q = self._qt_product(self.q, self._axis_angle_to_qt(dt*(w - self.bw)))

        # Update the predicted covariance
        Qi = np.copy(self._Qi)
        Qi[:6] = Qi[:6] * dt**2
        Qi[6:] = Qi[6:] * dt

        Fx = np.eye(self.P.shape[0])
        Fx[:3,3:6] = np.eye(3) * dt
        Fx[3:6,6:9] = - self._skew_symmetric(np.dot(C, a - self.ba)) * dt
        Fx[3:6,9:12] = - C*dt
        Fx[6:9,12:15] = - C*dt

        Fi = np.zeros((self.P.shape[0], self._Qi.shape[0]))
        Fi[3:,:] = np.eye(self._Qi.shape[0])

        self.P = np.dot(Fx, np.dot(self.P, Fx.T)) + np.dot(Fi, np.dot(Qi, Fi.T))

    def _H_roll_pitch(self):
        w, x, y, z = self.q

        H = np.zeros((2,17))
        H[0,6:10] = np.array([ 2*w*(-2*w*x - 2*y*z)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2) + 2*x*(w**2 - x**2 - y**2 + z**2)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2),
                               2*w*(w**2 - x**2 - y**2 + z**2)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2) - 2*x*(-2*w*x - 2*y*z)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2),
                              -2*y*(-2*w*x - 2*y*z)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2) + 2*z*(w**2 - x**2 - y**2 + z**2)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2),
                               2*y*(w**2 - x**2 - y**2 + z**2)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2) + 2*z*(-2*w*x - 2*y*z)/((2*w*x + 2*y*z)**2 + (w**2 - x**2 - y**2 + z**2)**2)])
        H[1,6:10] = np.array([-2*y/np.sqrt(1 - (2*w*y + 2*x*z)**2),
                              -2*z/np.sqrt(1 - (2*w*y + 2*x*z)**2),
                              -2*w/np.sqrt(1 - (2*w*y + 2*x*z)**2),
                              -2*x/np.sqrt(1 - (2*w*y + 2*x*z)**2)])

        return H

    def _H_yaw(self):
        w, x, y, z = self.q

        H = np.zeros((1,17))
        H[0,6:10] = np.array([2*w*(-2*w*z - 2*x*y)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2) + 2*z*(w**2 + x**2 - y**2 - z**2)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2),
                              2*x*(-2*w*z - 2*x*y)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2) + 2*y*(w**2 + x**2 - y**2 - z**2)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2),
                              2*x*(w**2 + x**2 - y**2 - z**2)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2) - 2*y*(-2*w*z - 2*x*y)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2),
                              2*w*(w**2 + x**2 - y**2 - z**2)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2) - 2*z*(-2*w*z - 2*x*y)/((2*w*z + 2*x*y)**2 + (w**2 + x**2 - y**2 - z**2)**2)])

        return H

    def _Xd(self):
        w, x, y, z = self.q
        Xd = np.zeros((17, 16))
        Xd[:6,:6] = np.eye(6)
        Xd[10:,9:] = np.eye(7)
        Xd[6:10,6:9] = np.array([[-x, -y, -z],
                                 [ w,  z, -y],
                                 [-z,  w,  x],
                                 [ y, -x,  w]]) / 2.
        return Xd

    def _kalman_gain(self, H, J):
        return np.dot(self.P, np.dot(H.T, np.linalg.inv(np.dot(H , np.dot(self.P, H.T)) + J)))

    def _correct_state_from_inno(self, dx):
        self.p = self.p + dx[:3]
        self.v = self.v + dx[3:6]
        self.q = self._qt_product(self.euler_to_qt(dx[6:9]), self.q)
        self.ba = self.ba + dx[9:12]
        self.bw = self.bw + dx[12:15]
        self.br = self.br + dx[15]

    def _correct_cov_sys(self, K, H, J):
        temp = np.eye(16) - np.dot(K,H)
        self.P = np.dot(temp, np.dot(self.P, temp.T)) + np.dot(K, np.dot(J, K.T))

    def _reset(self, dr):
        G = np.eye(16)
        G[9:12,9:12] = np.eye(3) + self._skew_symmetric(dr) / 2.

        self.P = np.dot(G, np.dot(self.P, G.T))

    def correct_position(self, gnss, J):
        H = np.dot(self._H_gnss, self._Xd())
        K = self._kalman_gain(H, J)
        inno = gnss - self.p
        dx = np.dot(K, inno)

        self._correct_state_from_inno(dx)
        self._correct_cov_sys(K, H, J)
        self._reset(dx[6:9])

    def correct_roll_pitch(self, a, J):
        x, y, z = (a - self.ba)

        rp = np.array([np.arctan2(y, z),
                       -np.arctan2(x, np.sqrt(y**2 + z**2))])

        m11 = 0.
        m12 = z / (y**2 + z**2)
        m13 = -y / (y**2 + z**2)
        norm = (x**2 + y**2 + z**2)
        m21 = -np.sqrt(y**2 + z**2) / norm
        m22 = x*y / norm / np.sqrt(y**2 + z**2)
        m23 = x*z / norm / np.sqrt(y**2 + z**2)
        M = np.array([[m11, m12, m13],
                      [m21, m22, m23]])
        J_rp = np.dot(M, np.dot(J, M.T))

        H = np.dot(self._H_roll_pitch(), self._Xd())
        K = self._kalman_gain(H, J_rp)
        inno = self._wrap_angle(rp - self.qt_to_euler(self.q)[:-1])
        dx = np.dot(K, inno)

        self._correct_state_from_inno(dx)
        self._correct_cov_sys(K, H, J_rp)
        self._reset(dx[6:9])

    def correct_yaw(self, h, J):
        x, y, z = h
        r, p, yaw = self.qt_to_euler(self.q)

        sr = np.sin(r)
        cr = np.cos(r)
        sp = np.sin(p)
        cp = np.cos(p)

        H1 = x*cp + y*sr*sp + z*cr*sp
        H2 = -y*cr + z*sr
        mag_yaw = np.arctan2(H2, H1) + self._D

        M1 = np.array([[-H2/(H1**2 + H2**2),
                         H1/(H1**2 + H2**2)]])
        M2 = np.array([[cp, sr*sp, cr*sp],
                       [0.,   -cr,    sr]])
        M = np.dot(M1, M2)
        J_y = np.dot(M, np.dot(J, M.T))

        H = np.dot(self._H_yaw(), self._Xd())
        K = self._kalman_gain(H, J_y)
        inno = self._wrap_angle(np.array([mag_yaw - yaw]))
        dx = np.dot(K, inno)

        self._correct_state_from_inno(dx)
        self._correct_cov_sys(K, H, J_y)
        self._reset(dx[6:9])

p0 = np.zeros(3); v0 = np.zeros(3); q0 = np.array([1., 0., 0., 0.])
ba0 = np.zeros(3); bw0 = np.zeros(3); r_eff = 2.0; g = np.array([0., 0., -9.81])
D = 0.81 * np.pi / 180.
P0 = np.eye(16)
var_a = 0.6; var_w = 0.1; var_ba = 0.05; var_bw = 0.01; var_br = 0.1;
J_roll_pitch = np.eye(3)*0.6
J_yaw = np.eye(3)*0.1

print("COMPILING ...")
print("Please Wait ...")
esekf = ESEKF_IMU(p0, v0, q0, ba0, bw0, r_eff, g, D, P0,
                  var_a, var_w, var_ba, var_bw, var_br)
_ = esekf.get_state()
_ = esekf.get_cov_sys()
_ = esekf.predict(0.01, np.array([0.1,0.2,9.78]), np.array([0.005,0.005,2.0]))
_ = esekf.correct_position(np.array([0.1, 0.1, 0.1]), np.eye(3) * 1.0)
_ = esekf.correct_roll_pitch(np.array([0.1,0.2,9.78]), J_roll_pitch)
_ = esekf.correct_yaw(np.array([0.1,0.2,9.78]), J_yaw)
print('Done !')
