import numpy as np
import numba as nb
import pymap3d as pm
from lib_ta_py.controller_2D import Controller_v1
import rospy
from pkg_ta.msg import Control
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

freq = 20 # Hz
waypoints_np = np.load('waypoints/waypoints/wp_24_agus_2.npy')

# In the Arduino, CW is positive and CCW is negative
# On the other hand, in the controller algoritm, CCW is positive and CW is negative
max_steer = 35.; min_steer = -28. # For the path following control algoritm ~ degree
max_steer_arduino = 28.; min_steer_arduino = -35. # For the Arduino ~ degree
max_brake = 2.9; max_throttle = 0.12

kp = 0.15; ki = 0.075; kd = 0.0
ff_long = np.array([0.0, 0.0]) # no feed-forward
ks = 1.5; kv = 2.; kff_lat = 2.3; dead_band_limit = 0.01
kv_lat = 0.5; kv_yaw = 0.5
sat_long = np.array([-np.abs(max_brake), np.abs(max_throttle)])
sat_lat = np.array([-np.abs(min_steer), np.abs(max_steer)])
sat_lat = sat_lat * np.pi / 180.

# Reference point
lat0, lon0, h0 = -6.8712, 107.5738, 768

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}

RUN_imu = False # Tunggu sampai data yaw masuk
RUN_gps = False # Tunggu sampai data posisi masuk
RUN_filtered_map = False # Tunggu sampai data kecepatan masuk

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
        state['yaw'] = euler[-1]
        
        RUN_imu = True
        
    def callback_filtered_map(msg_fm):
        global state
        global RUN_filtered_map
        
        vel = msg_fm.twist.twist.linear
        state['v'] = np.sqrt(vel.x**2 + vel.y**2) # m/s
        
        RUN_filtered_map = True

    # Create the controller object
    controller = Controller_v1(kp, ki, kd, ff_long, sat_long,
                           ks, kv, kff_lat, dead_band_limit, sat_lat,
                           waypoints_np)

    rospy.init_node('control')
    rospy.Subscriber('/fix', NavSatFix, callback_gps)
    rospy.Subscriber('/imu', Imu, callback_imu)
    rospy.Subscriber('/odometry/filtered_map', Odometry, callback_filtered_map)
    pub = rospy.Publisher('/control_signal', Control, queue_size=1)
    rate = rospy.Rate(freq) # Hz
            
    print("Menunggu data gps, imu, dan kecepatan masuk pertama kali ...")
    RUN = False
    while not RUN:
        RUN = RUN_gps and RUN_imu and RUN_filtered_map
        pass # INI DI WHILE KALAU DATA GPS BELUM MASUK BUAT INISIALISASI
    print("Data Navigasi sudah masuk !")
    print("Program sudah berjalan !")

    msg = Control()
    msg.header.frame_id = 'path_following_control'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq

    while not rospy.is_shutdown():
        # Calculate the actual sampling time
        msg.header.stamp = rospy.Time.now()
        delta_t = msg.header.stamp.to_sec() - last_time
        last_time = msg.header.stamp.to_sec()
        
        err = controller.get_error()
        lon = max_throttle / (1. + kv_yaw*np.abs(err[2]) + kv_lat*np.abs(err[1]))
        msg.action_throttle = max(lon, 0.075)

        # Calculate the control signal
        #long, lat = controller.calculate_control_signal(delta_t, state['x'],
        #                                                state['y'], state['v'],
        #                                                state['yaw'])
        long, lat = controller.calculate_control_signal(delta_t, state['x'],
                                                        state['y'], msg.action_throttle*10.,
                                                        state['yaw'])

        # Get the error profile
        err = controller.get_error()
        # Get the reference
        ref = controller.get_instantaneous_setpoint()
        
        # Send the message
        msg.header.seq += 1
        msg.action_steer = max(min(-lat*180/np.pi, max_steer_arduino), min_steer_arduino) # lat ~ radian
        
        lon = max_throttle / (1. + kv_yaw*np.abs(err[2]) + kv_lat*np.abs(err[1]))
        msg.action_throttle = max(lon, 0.075)
        #msg.action_brake = max(min(-long, max_brake), 0.)
        msg.action_brake = 0.
        
        msg.error_speed = err[0]
        msg.error_lateral = err[1]
        msg.error_yaw = err[2]        

        msg.actual_x = state['x']
        msg.actual_y = state['y']
        msg.actual_yaw = state['yaw']
        msg.actual_speed = state['v']
        
        msg.wp_idx = controller.get_closest_index()
        msg.ref_x = ref[0]
        msg.ref_y = ref[1]
        msg.ref_yaw = ref[2]
        msg.ref_speed = ref[3]
        msg.ref_curvature = ref[4]

        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
