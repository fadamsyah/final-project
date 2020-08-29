#!/usr/bin/env python
# coding: utf-8

# # Error State Extended Kalman Filter

import numpy as np

class ES_EKF():
    def __init__(self, freq, var_imu_a, var_imu_w, var_gnss, var_speed, var_compass):
        self.dt = 1 / freq
        self.g = np.array([0, 0, -9.81])

        # Matriks kovariansi noise
        # Q: Matriks kovariansi noise IMU
        # J_gnss: Matriks kovariansi noise GNSS
        # J_speed: Matriks kovariansi noise Wheel encoder
        self.var_a = var_imu_a
        self.var_w = var_imu_w
        self.Q = np.diag([self.var_a, self.var_a, self.var_a, self.var_w,  self.var_w,  self.var_w]) * self.dt**2
        J_gnss = np.diag([var_gnss, var_gnss, var_gnss])
        J_speed = np.array(var_speed).reshape(1,1)
        J_compass = np.array(var_compass).reshape(1,1)
        self.J = {'GNSS': J_gnss, 'SPEED': J_speed, 'COMPASS': J_compass}

        # Matriks Jacobian yang bernilai konstan
        # G: Matriks Jacobian noise pada model kinematik
        # H_gnss : Matriks Jacobian sensor GNSS
        self.G = np.zeros([9, 6])
        self.G[3:, :] = np.eye(6)
        H_gnss = np.zeros([3,9])
        H_gnss[:,:3] = np.eye(3,3)
        H_speed = np.zeros(9).reshape(1,9)
        self.H = {'GNSS': H_gnss, 'SPEED': H_speed}

        # Matriks Jacobian model eror terhadap eror
        self.F = np.eye(9,9)
        self.F[:3, 3:6] = self.dt * np.eye(3)
        # R : Matriks Jacobian reset
        self.R = np.zeros([9, 9])
        self.R[:6, :6] = np.eye(6)

    def change_dt(self, dt):
        self.dt = dt
        self.Q = np.diag([self.var_a, self.var_a, self.var_a, self.var_w,  self.var_w,  self.var_w]) * self.dt**2
        self.F[:3, 3:6] = self.dt * np.eye(3)

        return None

    def from_euler(self, r, p, y):
        # Convert euler angle to quaternion

        # Input :
        # r = roll
        # p = pitch
        # y = yaw

        # Output :
        # quaternion form in numpy array
        cr = np.cos(r/2)
        sr = np.sin(r/2)
        cp = np.cos(p/2)
        sp = np.sin(p/2)
        cy = np.cos(y/2)
        sy = np.sin(y/2)

        q1 = cr*cp*cy + sr*sp*sy
        q2 = sr*cp*cy - cr*sp*sy
        q3 = cr*sp*cy + sr*cp*sy
        q4 = cr*cp*sy - sr*sp*cy

        return np.array([q1, q2, q3, q4])

    def to_euler(self, q0, q1, q2, q3):
        # Convert quaternion to euler angle

        # Output :
        # euler angle (roll, pitch, yaw)

        r = np.arctan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1**2 + q2**2))
        p = np.arcsin(2 * (q0*q2 - q3*q1))
        y = np.arctan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2**2 + q3**2))

        return np.array([r, p, y])

    def from_axis_angle(self, a1, a2, a3):
        # Convert axis angle to quaternion

        # Output :
        # quaternion form in numpy array
        axis_angle = np.array([a1, a2, a3])
        norm = np.linalg.norm(axis_angle)
        q = np.zeros(4)

        q[0] = np.cos(norm/2)
        if norm < 10**(-10) :
            q[1:] = 0
        else :
            q[1:] = axis_angle / norm * np.sin(norm/2)

        return np.array(q)

    def skew_symmetric(self, a, b, c):
        #Matriks skew symmetric dari suatu vektor
        return np.array([ [0, -c, b], [c, 0, -a], [-b, a, 0] ])

    def product(self, left, right):
        # Quaternion product or multiplication
        qv = np.array([left[1], left[2], left[3]])
        m = np.zeros([4, 4])
        m[0,1:] = -qv
        m[1:,0] = qv
        m[1:,1:] = self.skew_symmetric(*qv)

        return np.array(np.dot(left[0] * np.eye(4) + m, right))

    def predict(self, p, v, q, P, a, w):
        # p: vektor posisi
        # v: vektor kecepatan
        # P: matriks kovariansi variabel keadaan error
        # a: percepatan hasil pengukuran IMU
        # w: kecepatan sudut hasil pengukuran IMU

        # Rotation Matrix
        qv = q[1:].reshape(3,1)
        C = np.array((q[0]**2 - qv.T @ qv) * np.eye(3) + 2 * qv @ qv.T + 2 * q[0] * self.skew_symmetric(*qv)).reshape(3,3)

        # Update state nominal
        p_new = p + self.dt * v + (self.dt**2)/2 * (C @ a + self.g)
        v_new = v + self.dt * (C @ a + self.g)
        q_new = self.product(q, self.from_axis_angle(*w * self.dt))

        # Matriks Jacobian F
        self.F[3:6, 6:] = - self.dt * self.skew_symmetric(*(C @ a))

        # Prediksi matriks kovariansi
        P_new = self.F @ P @ self.F.T + self.G @ self.Q @ (self.G).T

        return p_new, v_new, q_new, P_new

    def measured_variable_prediction(self, sensor, p, v, q):
        if (sensor.upper() == 'GNSS'):
            y = p
        elif (sensor.upper() == 'SPEED'):
            y = np.sqrt(np.sum(np.square(v)))
        elif (sensor.upper() == 'COMPASS'):
            y = self.to_euler(*q)[2]

        return y

    def inovation(self, sensor, y, p, v, q):
        if (sensor.upper() == 'GNSS'):
            z = y - p
        elif (sensor.upper() == 'SPEED'):
            z = (y - np.sqrt(np.sum(np.square(v))))
        elif (sensor.upper() == 'COMPASS'):

            z = np.remainder(y - self.to_euler(*q)[2], 2*np.pi)
            if z > np.pi:
                z -= 2*np.pi
            elif z <= -np.pi:
                z += 2*np.pi

        return z

    def measurement_jacobian(self, sensor, p, v, q):
        if (sensor.upper() == 'SPEED'):
            H = self.H[sensor.upper()]
            vv = np.sqrt(np.sum(np.square(v))) + 10**(-10)
            H[0,3] = v[0]/vv
            H[0,4] = v[1]/vv
            H[0,5] = v[2]/vv
        elif (sensor.upper() == 'GNSS'):
            H = self.H[sensor.upper()]
        elif (sensor.upper() == 'COMPASS'):
            dq0 = 2*q[3]*(-2*q[2]**2 - 2*q[3]**2 + 1)/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2)
            dq1 = 2*q[2]*(-2*q[2]**2 - 2*q[3]**2 + 1)/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2)
            dq2 = 2*q[1]*(-2*q[2]**2 - 2*q[3]**2 + 1)/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2) \
                    - 4*q[2]*(-2*q[0]*q[3] - 2*q[1]*q[2])/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2)
            dq3 = 2*q[0]*(-2*q[2]**2 - 2*q[3]**2 + 1)/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2) \
                    - 4*q[3]*(-2*q[0]*q[3] - 2*q[1]*q[2])/((2*q[0]*q[3] + 2*q[1]*q[2])**2 + (-2*q[2]**2 - 2*q[3]**2 + 1)**2)
            Hx = np.zeros(10).reshape(1,10)
            Hx[0,6] = dq0
            Hx[0,7] = dq1
            Hx[0,8] = dq2
            Hx[0,9] = dq3
            X = np.zeros((10,9))
            X[:6, :6] = np.eye(6)
            X[6:, 6:] = 0.5 * np.array([[-q[1], -q[2], -q[3]],
                        [q[0], q[3], -q[2]], [-q[3], q[0], q[1]],
                        [q[2], -q[1], q[0]]])
            H = np.dot(Hx, X)

        return H

    def correct(self, sensor, y, p, v, q, P):
        H = self.measurement_jacobian(sensor, p, v, q)
        K = (P @ H.T) @ np.linalg.inv(H @ P @ H.T + self.J[sensor.upper()])

        dx = np.dot(K,  self.inovation(sensor, y, p, v, q).reshape([H.shape[0],]))

        p_new = p + dx[:3]
        v_new = v + dx[3:6]
        q_new = self.product(self.from_euler(*dx[6:]), q)

        P_new = (np.eye(9) - K @ H) @ P @ (np.eye(9) - K @ H).T + K @ self.J[sensor.upper()] @ K.T
        P_new = self.reset_step(dx[6:], P_new)

        return p_new, v_new, q_new, P_new

    def reset_step(self, d, P):
        # d : delta theta
        self.R[6:, 6:] = np.eye(3,3) + 0.5 * self.skew_symmetric(*d)
        try:
            P_new = np.matmul(np.matmul(self.R, P), self.R.T)
        except:
            P_new = P
            print('Error in np.matmul(np.matmul(self.R, P), self.R.T)')

        return P_new
