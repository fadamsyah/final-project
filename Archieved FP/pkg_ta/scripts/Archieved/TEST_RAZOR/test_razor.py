import numpy as np
import numba as nb
import time
import rospy
from sensor_msgs.msg import Imu
from pkg_ta.msg import State_EKF_2D

freq = 20

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

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
pub = rospy.Publisher('/razor_yaw', State_EKF_2D, queue_size=1)
rate = rospy.Rate(freq) # Hz

msg = State_EKF_2D()

def callback_imu(msg_imu):
    q = msg_imu.orientation
    euler = to_euler(q.x, q.y, q.z, q.w)
    imu_yaw = euler[-1]
    msg.yaw_imu = euler[-1] * 180. / np.pi

# rospy.Subscriber('/android/imu', Imu, callback_imu)
rospy.Subscriber('/imu', Imu, callback_imu)

while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()

rospy.spin()
