import numpy as np
from numba import float64, int64
from numba.experimental import jitclass

spec = [('_a_scale', float64[:]), ('_a_bias', float64[:]),
        ('_mag_b', float64[:]), ('_mag_A', float64[:,:])]

@jitclass(spec)
class IMU_9_DOF_CALIBRATION(object):
    def __init__(self, a_min, a_max, mag_b, mag_A, g = 9.80665):
        self._a_scale = g / ((np.abs(a_min) + np.abs(a_max)) / 2.0)
        self._a_bias = (a_min + a_max) / 2.0 * self._a_scale
        self._mag_b = mag_b
        self._mag_A = mag_A

    def get_a_bias_init(self):
        return self._a_bias

    def calib_acc(self, a):
        return a * self._a_scale

    def calib_mag(self, m):
        return np.dot(self._mag_A, m - self._mag_b)

a_min = np.array([-9., -9., -9.])
a_max = np.array([10., 10., 10.])
mag_b = np.array([20., 10., 15.])
mag_A = np.eye(3)

Imu_Calib = IMU_9_DOF_CALIBRATION(a_min, a_max, mag_b, mag_A)
_ = Imu_Calib.get_a_bias_init()
_ = Imu_Calib.calib_acc(np.array([0.1, -0.2, 9.5]))
_ = Imu_Calib.calib_mag(np.array([100., -4., 54.]))

# print(Imu_Calib.get_a_bias_init())
# print(Imu_Calib.calib_acc(np.array([0.1, -0.2, 9.5])))
# print(Imu_Calib.calib_mag(np.array([100., -4., 54.])))
