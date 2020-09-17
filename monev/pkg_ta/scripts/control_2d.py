import numpy as np
import rospy
import time
import sys
from lib.stanley_2d import Controller
from pkg_ta.msg import Control
from pkg_ta.msg import State_EKF_2D

freq = 20 # Hz
# waypoints_np = np.load('waypoints/waypoints/31_agus_wp_lurus.npy')
# waypoints_np = np.load('waypoints/waypoints/10_09_wp_lurus.npy')
waypoints_np = np.load('waypoints/waypoints/10_09_wp_S.npy')
# waypoints_np = np.load('waypoints/waypoints/10_09_wp_belok.npy')
# waypoints_np = np.load('waypoints/waypoints/10_09_wp_belok_besar.npy')

# In the Arduino, CW is positive and CCW is negative
# On the other hand, in the controller algoritm, CCW is positive and CW is negative
max_steer = 35.; min_steer = -28. # For the path following control algoritm ~ degree
max_steer_arduino = 28.; min_steer_arduino = -35. # For the Arduino ~ degree
max_brake = 2.9; max_throttle = 0.25; min_throttle = 0.0; min_throttle_move = 0.08
min_vel_move = 0.5 # m/s

kp = 0.11; ki = 0.3; kd = 0.015
ff_long = np.array([0.0, 0.0]) # no feed-forward
ks = 0.75; kv = 1.0; kff_lat = 1.7; dead_band_lat = 0.025
kv_lat = 1.0; kv_yaw = 4.0; # Speed / Throttle Additional Control
sat_long = np.array([-np.abs(max_brake), np.abs(max_throttle)])
sat_lat = np.array([-np.abs(min_steer), np.abs(max_steer)])
sat_lat = sat_lat * np.pi / 180.

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}
RUN = False

def main():
    global RUN

    # Create the controller object
    controller = Controller(kp, ki, kd, ff_long, sat_long,
                            ks, kv, kff_lat, dead_band_lat, sat_lat,
                            waypoints_np, min_vel_move,
                            max_throttle, min_throttle_move,
                            kv_yaw, kv_lat)

    def callback(msg_nav):
        global state
        global RUN

        state['x'] = msg_nav.px
        state['y'] = msg_nav.py
        state['v'] = np.sqrt(msg_nav.vx**2 + msg_nav.vy**2)
        state['yaw'] = msg_nav.yaw
        # state['yaw'] = msg_nav.yaw_imu

        RUN = True

    rospy.init_node('control')
    rospy.Subscriber('/state_2d_new', State_EKF_2D, callback)
    pub = rospy.Publisher('/control_signal', Control, queue_size=1)
    rate = rospy.Rate(freq) # Hz

    print("Menunggu data navigasi masuk pertama kali ...")
    RUN = False
    while not RUN:
        time.sleep(0.02) # 20 ms
        pass

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
        msg.action_throttle = max(min(long, max_throttle), min_throttle)
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

        msg.deg_ref_yaw = msg.ref_yaw * 180. / np.pi
        msg.deg_actual_yaw = msg.actual_yaw * 180. / np.pi
        msg.deg_error_yaw = msg.error_yaw * 180. / np.pi

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
