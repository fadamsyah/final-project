import numpy as np
import numba as nb
import pymap3d as pm
import time
import rospy
import sys
import os
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from pkg_ta.msg import State_EKF_2D

sys.path.append(os.path.abspath(sys.path[0] + '/../src/Python'))
from ekf_2d_imu import imu_ekf_2d

# Frame
yawc_compass = - np.pi/2 # CEK LAGI MANA YANG BENER !
gnss_compass_var_max = 0.02
gnss_compass_vel_min = 0.25 # m/s
stop_vel_max = 0.125 # m/s

var_a = 0.5
var_w = 0.1
var_ba = 0.5
var_bw = 0.05
Q = np.zeros((6,6))
Q[:2,:2] = np.eye(2) * var_a
Q[2,2] = var_w
Q[3:5, 3:5] = np.eye(2) * var_ba
Q[5, 5] = var_bw

J_gnss = np.eye(2) * 0.01 # STANLEY
J_compass = np.array([[0.1]])
J_gnss_compass = np.array([[0.01]])
J_yaw_stop = np.array([[0.1]])
P0 = np.ones((8, 8))*0.3

ba0 = np.array([-0.5, 0.4])
bw0 = 0.02
ekf = imu_ekf_2d(np.zeros(2), np.zeros(2), 0.0, ba0, bw0, P0, Q, yawc_imu)

# Reference point
lat0, lon0, h0 = -6.8712, 107.5738, 768

@nb.njit()
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
_ = wrap_angle(1)
_ = wrap_angle(0.1)

@nb.njit()
def to_euler(x, y, z, w):
    """Dari Coursera: Return as xyz (roll pitch yaw) Euler angles."""
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([roll, pitch, yaw])
# Compile the to_euler
_ = to_euler(1.5352300785980803e-15, -1.3393747145983517e-15, -0.7692164172827881, 0.638988343698562)

rospy.init_node('ekf_imu_2d', anonymous=True)
pub = rospy.Publisher('/state_2d_new', State_EKF_2D, queue_size=1)

msg = State_EKF_2D()
msg.header.frame_id = 'state_ekf_imu_2d'
msg.header.seq = 0
msg.header.stamp = rospy.Time.now()

RUN_imu = False # Tunggu sampai data yaw masuk
RUN_gnss = False # Tunggu sampai data gnss masuk
RUN = RUN_imu and RUN_gnss

last_time = 0.0
arr_pos = np.zeros((2, 2))
arr_cov = np.zeros(2)
dydx_last = 0.0
def callback_gnss(msg_gnss):
    global RUN, RUN_imu, RUN_gnss
    global arr_pos, arr_cov
    global dydx_last

    gnss_pos = np.array(pm.geodetic2enu(msg_gnss.latitude,
                                        msg_gnss.longitude,
                                        msg_gnss.altitude,
                                        lat0, lon0, h0))

    if not RUN:
        # Initial State
        ekf.p[0] = gnss_pos[0]
        ekf.p[1] = gnss_pos[1]
        arr_pos[-1] = np.copy(arr_pos[0])
        arr_pos[0] = np.array([gnss_pos[0], gnss_pos[1]])
        arr_cov[-1] = np.copy(arr_cov[0])
        arr_cov[0] = msg_gnss.position_covariance[0]
    else:
        ekf.correct_gnss_2d(gnss_pos[:-1], J_gnss)
        if np.linalg.norm(ekf.v) >= gnss_compass_vel_min: # m/s
            n = 0
            diff = np.array([gnss_pos[0], gnss_pos[1]]) - arr_pos[n]
            gnss_compass = np.arctan2(diff[-1], diff[0])
            if np.abs(gnss_compass - dydx_last) <= 25./180.*np.pi: # Cancel the noise
                ekf.correct_compass(gnss_compass, J_gnss_compass)
            dydx_last = np.copy(gnss_compass) + 0.0
        elif np.linalg.norm(ekf.v) <= stop_vel_max: # m/s
            ekf.correct_compass(ekf.yaw, J_yaw_stop)
        arr_pos[-1] = np.copy(arr_pos[0])
        arr_pos[0] = np.array([gnss_pos[0], gnss_pos[1]])
        arr_cov[-1] = np.copy(arr_cov[0])
        arr_cov[0] = msg_gnss.position_covariance[0]
    RUN_gnss = True

def callback_imu(msg_imu):
    global last_time
    global RUN, RUN_imu, RUN_gnss

    imu_stamp = msg_imu.header.stamp
    dt = imu_stamp.to_sec() - last_time
    last_time = imu_stamp.to_sec()

    q = msg_imu.orientation
    euler = to_euler(q.x, q.y, q.z, q.w)
    imu_yaw = euler[-1]
    imu_a = msg_imu.linear_acceleration
    imu_w = msg_imu.angular_velocity

    if not RUN:
        # Initial State
        ekf.yaw = wrap_angle(imu_yaw - yawc_compass)
    else:
        ekf.predict(dt, np.array([imu_a.x, -imu_a.y]), -imu_w.z) # Mounted Razor IMU
        # ekf.correct_compass(wrap_angle(imu_yaw - yawc_compass), J_compass) # Gapake compass dulu

        msg.header.seq = msg.header.seq + 1
        msg.header.stamp = rospy.Time.now()
        msg.px, msg.py = ekf.p
        msg.vx, msg.vy = ekf.v
        msg.yaw = ekf.yaw
        msg.bax, msg.bay = ekf.ba
        msg.bw = ekf.bw
        msg.P = ekf.P.flatten()
        msg.yaw_imu = wrap_angle(imu_yaw - yawc_compass)
        pub.publish(msg)

    RUN_imu = True

rospy.Subscriber('/fix', NavSatFix, callback_gnss)
rospy.Subscriber('/imu', Imu, callback_imu)

print("Menunggu data gnss dan IMU masuk pertama kali ...")
while not RUN:
    RUN = RUN_gnss and RUN_imu
    time.sleep(0.02) # 20 ms
    pass
print("Data Navigasi sudah masuk !")
print("Program sudah berjalan !")

rospy.spin()
