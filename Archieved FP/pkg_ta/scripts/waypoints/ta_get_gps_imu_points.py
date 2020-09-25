import numpy as np
import numba as nb
import rospy
import pymap3d as pm
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from pkg_ta.msg import State

freq = 10 # Hz

# Reference point
lat0, lon0, h0 = -6.8712, 107.5738, 768

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}

RUN_imu = False # Tunggu sampai data yaw masuk
RUN_gps = False # Tunggu sampai data posisi masuk

@nb.njit()
def to_euler(x, y, z, w):
    """Dari Coursera: Return as xyz (roll pitch yaw) Euler angles."""
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([roll, pitch, yaw])
# Compile the to_euler
_ = to_euler(1.5352300785980803e-15, -1.3393747145983517e-15, -0.7692164172827881, 0.638988343698562)

def main():
    def callback_gps(msg_gps):
        global state
        global RUN_gps
        
        gps_pos = pm.geodetic2enu(msg_gps.latitude,
                                  msg_gps.longitude,
                                  msg_gps.altitude, lat0, lon0, h0)
        state['x'] = gps_pos[0]
        state['y'] = gps_pos[1]
        
        RUN_gps = True
    
    def callback_imu(msg_imu): # Callback IMU
        global state
        global RUN_imu
        
        q = msg_imu.orientation
        euler = to_euler(q.x, q.y, q.z, q.w)
        state['yaw'] = euler[-1] # * (-1) # Dikali mines nya di generate waypoints yakk
        
        RUN_imu = True
    
    rospy.init_node('get_gps_imu_points')
    rospy.Subscriber('/imu', Imu, callback_imu)
    rospy.Subscriber('/fix', NavSatFix, callback_gps)
    pub = rospy.Publisher('/get_waypoints', State, queue_size=1)
    rate = rospy.Rate(freq) # Hz
            
    print("Menunggu data posisi dan yaw masuk pertama kali ...")
    RUN = False
    while not RUN:
        RUN = RUN_gps and RUN_imu
        pass # INI DI WHILE KALAU DATA GPS BELUM MASUK BUAT INISIALISASI
    print("Data Navigasi sudah masuk !")
    print("Program sudah berjalan !")
    
    msg = State()
    msg.header.frame_id = 'get_waypoint_from_gps_imu'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # Send the message
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        msg.x = state['x']
        msg.y = state['y']
        msg.yaw = state['yaw']
        
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
