import numpy as np
from numba import float64, int64
from numba.experimental import jitclass

spec = [('p', float64[:]), ('v', float64[:]), ('rpy', float64[:]),
        ('ba', float64[:]), ('bw', float64[:]), ('br', float64), ('P', float64[:, :]),
        ('_N', int64), ('_r_eff', float64), ('_g', float64[:]), ('_Q', float64[:, :]),
        ('_lambda', float64), ('_Wm', float64[:]), ('_Wc', float64[:])]

@jitclass(spec)
class UKF_IMU(object):
    def __init__(self, p0, v0, rpy0, ba0, bw0, r_eff, g, P0,
                 var_a, var_w, var_ba, var_bw, var_br,
                 alpha, kappa, beta):
        self.p = p0
        self.v = v0
        self.rpy = rpy0
        self.ba = ba0
        self.bw = bw0
        self.br = 0.0
        self.P = P0
        self._N = self.P.shape[0]
        self._r_eff = r_eff
        self._g = g

        self._Q = np.zeros((13,13))
        self._Q[:3,:3] = np.eye(3) * var_a
        self._Q[3:6,3:6] = np.eye(3) * var_w
        self._Q[6:9,6:9] = np.eye(3) * var_ba
        self._Q[9:12,9:12] = np.eye(3) * var_bw
        self._Q[12,12] = var_br

        self._lambda = alpha**2 * (self._N + kappa) - self._N
        self._Wm = 1. / (2.*(self._N + self._lambda)) * np.ones(2*self._N + 1)
        self._Wm[0] = self._lambda / (self._N + self._lambda)
        self._Wc = 1. / (2.*(self._N + self._lambda)) * np.ones(2*self._N + 1)
        self._Wc[0] = self._lambda / (self._N + self._lambda) + (1 - alpha**2 + beta)

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _to_rot_mat(self, rpy):
        r, p, y = rpy

        sr = np.sin(r)
        cr = np.cos(r)
        sp = np.sin(p)
        cp = np.cos(p)
        sy = np.sin(y)
        cy = np.cos(y)

        return np.array([[cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                         [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                         [-sp, sr*cp, cr*cp]])


    def _euler_kinematic_matrix(self, rpy):
        r, p, y = rpy

        sr = np.sin(r)
        cr = np.cos(r)
        sp = np.sin(p)
        cp = np.cos(p)
        tp = np.tan(p)

        return np.array([[1., sr*tp, cr*tp],
                         [0., cr, -sr],
                         [0., sr/cp, cr/cp]])

    def get_state(self):
        return self.p, self.v, self.rpy, self.ba, self.bw, self.br

    def get_cov_sys(self):
        return self.P

    def predict(self, dt, a, w):
        # Generate Sigma Points
        L = np.linalg.cholesky(self.P)
        xs = np.zeros((self._N, 2*self._N + 1))
        xs[:, 0] = np.concatenate((self.p, self.v, self.rpy,
                                   self.ba, self.bw, np.array([self.br])))
        for i in range(1, self._N+1):
            xs[:, i] = xs[:, 0] + np.sqrt(self._N + self._lambda) * L[:, i-1]
            xs[:, i + self._N] = xs[:, 0] - np.sqrt(self._N + self._lambda) * L[:, i-1]

        # Propagate Sigma Points
        xs_prop = np.empty_like(xs)
        for i in range(2*self._N + 1):
            C = self._to_rot_mat(xs[6:9, i])
            xs_prop[:3, i] = xs[:3, i] + dt * xs[3:6, i] + dt**2 * (np.dot(C, a - xs[9:12, i]) + self._g) / 2
            xs_prop[3:6, i] = xs[3:6, i] + dt * (np.dot(C, a - xs[9:12, i]) + self._g)
            xs_prop[6:9, i] = xs[6:9, i] + dt * np.dot(self._euler_kinematic_matrix(xs[6:9, i]), w - xs[12:15, i])
            xs_prop[9:, i] = xs[9:, i] # The bias remains the same in this propagation of prediction step

        # Compute the mean and predicted state
        x_mean = np.sum(xs_prop * self._Wm, axis=-1)

        self.p = x_mean[:3]
        self.v = x_mean[3:6]
        self.rpy = self._wrap_angle(x_mean[6:9])
        self.ba = x_mean[9:12]
        self.bw = x_mean[12:15]
        self.br = x_mean[15]

        # Compute the predicted covariance
        self.P = np.zeros((self._N, self._N))
        for i in range(2*self._N + 1):
            temp = xs_prop[:,i] - x_mean
            self.P += self._Wc[i] * np.outer(temp, temp)
        L = np.zeros((x_mean.shape[0], self._Q.shape[0]))
        L[:3,:3] = dt**2 * C / 2
        L[3:6,:3] = dt * C
        L[6:9,3:6] = dt * self._euler_kinematic_matrix(self.rpy)
        L[9:,6:] = np.eye(L[9:,6:].shape[0])
        Q_temp = np.copy(self._Q)
        Q_temp[:6,:6] = Q_temp[:6,:6] #* dt**2
        Q_temp[6:,6:] = Q_temp[6:,6:] * dt
        self.P += np.dot(np.dot(L, Q_temp), L.T)

p0 = np.zeros(3); v0 = np.zeros(3); rpy0 = np.zeros(3)
ba0 = np.zeros(3); bw0 = np.zeros(3); r_eff = 2.0; g = np.array([0., 0., -9.81])
P0 = np.eye(16)
var_a = 0.6; var_w = 0.1; var_ba = 0.05; var_bw = 0.01; var_br = 0.1;
alpha = 1e-3; kappa = 0.; beta = 2.

ukf = UKF_IMU(p0, v0, rpy0, ba0, bw0, r_eff, g, P0,
              var_a, var_w, var_ba, var_bw, var_br,
              alpha, kappa, beta)

print("COMPILING ...")
print("Please Wait ...")
_ = ukf.get_state()
_ = ukf.get_cov_sys()
_ = ukf.predict(0.01, np.array([0.1,0.2,9.78]), np.array([0.005,0.005,2.0]))
print('Done !')