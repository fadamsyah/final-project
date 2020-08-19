import numpy as np
import pandas as pd
import numba as nb
from lib_ta_py.controller_2D import Controller_v1
import rospy
from pkg_ta.msg import Control
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

freq = 10 # Hz
waypoints_np = np.load(.............)

# In the Arduino, CW is positive and CCW is negative
# On the other hand, in the controller algoritm, CCW is positive and CW is negative
max_steer = 35.; min_steer = -28. # For the path following control algoritm ~ degree
max_steer_arduino = 28.; min_steer_arduino = -35. # For the Arduino ~ degree
max_brake = 2.9; max_throttle = 0.125

kp = 0.15; ki = 0.075; kd = 0.0
ff_long = np.array([0.0, 0.0]) # no feed-forward
sat_long = np.array([-np.abs(max_brake), np.abs(max_throttle)])
sat_lat = np.array([-np.abs(min_steer), np.abs(max_steer)])
sat_lat = sat_lat * np.pi / 180.

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}

RUN_imu = False # Tunggu sampai data yaw masuk
RUN_utm = False # Tunggu sampai data posisi masuk
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
    # Create the controller object
    controller = Controller_v1(kp, ki, kd, ff_long, sat_long,
                           2.0, 1.0, 2.3, 0.01, sat_lat,
                           waypoints_np)
    
    def callback_utm(msg_utm): # Callback UTM
        global state
        global RUN_utm
        
        pos = msg_utm.pose.pose.position
        state['x'] = pos.x
        state['y'] = pos.y
        
        RUN_utm = True
    
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

    rospy.init_node('control')
    rospy.Subscriber('/odometry/utm', Odometry, callback_utm)
    rospy.Subscriber('/imu', Imu, callback_imu)
    rospy.Subscriber('/odometry/filtered_map', Odometry, callback_filtered_map)
    pub = rospy.Publisher('/control_signal', Control, queue_size=1)
    rate = rospy.Rate(freq) # Hz
            
    print("Menunggu data posisi, kecepatan, dan yaw masuk pertama kali !")
    RUN = False
    while not RUN:
        RUN = RUN_utm and RUN_imu and RUN_filtered_map
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
        
        # Calculate the control signal
        long, lat = controller.calculate_control_signal(delta_t, state['x'],
                                                        state['y'], state['v'],
                                                        state['yaw'])

        # Get the error profile
        err = controller.get_error()
        # Get the reference
        ref = controller.get_instantaneous_setpoint()
        
        # Send the message
        msg.header.seq += 1
        msg.action_steer = max(min(-lat*180/np.pi, max_steer_arduino), min_steer_arduino) # lat ~ radian
        msg.action_throttle = max(min(long, max_throttle), 0.)
        msg.action_brake = max(min(-long, max_brake), 0.)
        
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
