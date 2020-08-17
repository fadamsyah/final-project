import numpy as np
import numba as nb
from numba import jitclass, f8

# BELUM DIPAKEIN NUMBA
# CORRECT OMEGA BELUM
# BELUM PREDIKSI ANGULAR ACCELERATION (GATAU PERLU APA NGGA)
# BELUM NAMBAHIN RESET STATE DAN COVARIANCE MATRIX

spec = [('cov_gps_pos', f8[:,:]), ('cov_gps_vel', f8[:,:]), ('cov_gps_yaw', f8),
        ('H_pos', f8[:,:]), ('H_vel', f8[:,:]), ('H_yaw', f8[:,:]), ('Q', f8[:,:]),
        ('x', f8[:]), ('v', f8[:]), ('a', f8[:]), ('yaw', f8), ('w', f8), ('P', f8[:,:])]

@jitclass(spec)
class KF_gps(object):
    def __init__(self, var_gps_pos, var_gps_speed, var_gps_yaw, Q,
                x0, v0, a0, yaw0, w0, P0):
        # Measurement Covariance Matrix
        self.cov_gps_pos = np.array([[1,0],
                                     [0,1]]) * var_gps_pos
        self.cov_gps_vel = np.eye(2) * var_gps_speed
        self.cov_gps_yaw = var_gps_yaw
        
        # Measurement Jacobian Matrix
        self.H_pos = np.zeros((2,8))
        self.H_pos[:2,:2] = np.eye(2)
        self.H_vel = np.zeros((2,8))
        self.H_vel[:2,2:4] = np.eye(2)
        self.H_yaw = np.zeros((1,8))
        self.H_yaw[0, -2] = 1
        
        # Process Noise Covariance Matrix
        self.Q = Q
        
        # Initial state
        self.x = x0 # (2,) Position
        self.v = v0 # (2,) Velocity
        self.a = a0 # (2,) Linear Velocity
        self.yaw = yaw0 # Yaw
        self.w = w0 # Angular rate
        
        # Initial State Covariance Matrix
        self.P = P0
    
    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
        
    def predict(self, dt):
        # Predict the next state
        self.x = self.x + self.v*dt + self.a*dt**2 / 2
        self.v = self.v + self.a*dt
        #self.a = np.copy(self.a)
        self.yaw = self.wrap_angle(self.yaw + self.w*dt)
        #self.w = np.copy(self.w)
        
        # State Jacobian Matrix
        F = np.eye(8)
        F[:2,2:4] = np.eye(2) * dt
        F[:2,4:6] = np.eye(2) * dt**2 / 2
        F[2:4,4:6] = np.eye(2) * dt
        F[6,-1] = dt
        
        # Update the covariance matrix
        self.P = F @ self.P @ F.T + self.Q * dt**2
        return self.x, self.v, self.a, self.yaw, self.w, self.P
    
    def correct_position(self, z):
        H = self.H_pos
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @H.T + self.cov_gps_pos)
        inno = z - self.x
        
        self.x = self.x + K[:2] @ inno
        self.v = self.v + K[2:4] @ inno
        self.a = self.a + K[4:6] @ inno
        self.yaw = self.wrap_angle(self.yaw + K[6] @ inno)
        self.w = self.w + K[7] @ inno
        
        self.P = (np.eye(8) - K @ H) @ self.P
        return self.x, self.v, self.a, self.yaw, self.w, self.P
    
    def correct_velocity(self, z):
        H = self.H_vel
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @H.T + self.cov_gps_vel)
        inno = z - self.v
        
        self.x = self.x + K[:2] @ inno
        self.v = self.v + K[2:4] @ inno
        self.a = self.a + K[4:6] @ inno
        self.yaw = self.wrap_angle(self.yaw + K[6] @ inno)
        self.w = self.w + K[7] @ inno
        
        self.P = (np.eye(8) - K @ H) @ self.P
        return self.x, self.v, self.a, self.yaw, self.w, self.P
    
    def correct_yaw(self, z):
        H = self.H_yaw
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @H.T + self.cov_gps_yaw)
        inno = self.wrap_angle(np.array([z - self.yaw]))
        
        self.x = self.x + K[:2] @ inno
        self.v = self.v + K[2:4] @ inno
        self.a = self.a + K[4:6] @ inno
        self.yaw = self.wrap_angle(self.yaw + K[6] @ inno)
        self.w = self.w + K[7] @ inno
        
        self.P = (np.eye(8) - K @ H) @ self.P
        return self.x, self.v, self.a, self.yaw, self.w, self.P
    
print("Compilling the KF_gps class. Please wait ...")
kf = KF_gps(0.1, 0.1, 0.1, np.eye(8, dtype=float),
           np.array([0.1,0.1]), np.array([0.1,0.1]), np.array([0.1,0.1]), 0.3, 0.2, np.eye(8, dtype=float))
_ = kf.predict(0.01)
_ = kf.correct_position(np.array([1., 1.]))
_ = kf.correct_velocity(np.array([1., 1.]))
_ = kf.correct_yaw(0.2)
print("The KF_gps class has been compiled !")