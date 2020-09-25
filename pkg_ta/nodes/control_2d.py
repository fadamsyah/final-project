#!/usr/bin/env python3

import numpy as np
import rospy
import time
import sys
import os
from pkg_ta.msg import Control
from pkg_ta.msg import State_EKF_2D

sys.path.append(os.path.abspath(sys.path[0] + '/../src/Python'))
from stanley_2d import Controller

rospy.init_node('control')

# The actual state
state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}
RUN = False

# Receive the estimated state
def callback(msg_nav):
    global state
    global RUN

    state['x'] = msg_nav.px
    state['y'] = msg_nav.py
    state['v'] = np.sqrt(msg_nav.vx**2 + msg_nav.vy**2)
    state['yaw'] = msg_nav.yaw
    # state['yaw'] = msg_nav.yaw_imu

    RUN = True

freq = rospy.get_param('~freq', 20.) # Hz
ff_1 = rospy.get_param('~ff_1', 0.0)
ff_2 = rospy.get_param('~ff_2', 0.0)
kp = rospy.get_param('~kp', 0.11)
ki = rospy.get_param('~ki', 0.30)
kd = rospy.get_param('~kd', 0.015)
sat_long_max = rospy.get_param('~sat_long_max', 0.3)
sat_long_min = rospy.get_param('~sat_long_min', -2.9)
kv_yaw = rospy.get_param('~kv_yaw', 2.25)
kv_lat = rospy.get_param('~kv_lat', 0.75)
min_vel_move = rospy.get_param('~min_vel_move', 0.5)
max_throttle_move = rospy.get_param('~max_throttle_move', 0.3)
min_throttle_move = rospy.get_param('~min_throttle_move', 0.3)
length = rospy.get_param('~length', 1.7)
ks = rospy.get_param('~ks', 0.75)
kv = rospy.get_param('~kv', 1.00)
lateral_dead_band = rospy.get_param('~lateral_dead_band', 0.025)
sat_lat_max = rospy.get_param('~sat_lat_max', 0.6109)
sat_lat_min = rospy.get_param('~sat_lat_min', -0.4887)
waypoints_path = rospy.get_param('~waypoints_path', 'wp_monev_baru.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)

feed_forward_params = np.array([ff_1, ff_2])
sat_long = np.array([sat_long_min, sat_long_max])
sat_lat = np.array([sat_lat_min, sat_lat_max])
waypoints = np.load(waypoints_path)

# Create the controller object
controller = Controller(kp, ki, kd, feed_forward_params, sat_long,\
                        ks, kv, length, lateral_dead_band, sat_lat,\
                        waypoints, min_vel_move,\
                        max_throttle_move, min_throttle_move, kv_yaw, kv_lat)

rospy.Subscriber('/state_2d_new', State_EKF_2D, callback)
pub = rospy.Publisher('/control_signal', Control, queue_size=1)
rate = rospy.Rate(freq) # Hz

# Wait until we get the actual state
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
    ### Calculate the actual sampling time
    msg.header.stamp = rospy.Time.now()
    delta_t = msg.header.stamp.to_sec() - last_time
    last_time = msg.header.stamp.to_sec()

    ### Calculate the control signal
    long, lat = controller.calculate_control_signal(delta_t, state['x'],
                                                    state['y'], state['v'],
                                                    state['yaw'])

    ### Get the error profile
    err = controller.get_error()
    ### Get the reference
    ref = controller.get_instantaneous_setpoint()

    ### Send the message
    # Header
    msg.header.seq += 1
    # Cotrol action
    msg.action_steer = max(min(-lat*180/np.pi, max_steer_arduino), min_steer_arduino) # lat ~ (rad)
    msg.action_throttle = max(min(long, max_throttle), min_throttle)
    #msg.action_brake = max(min(-long, max_brake), 0.)
    msg.action_brake = 0.
    # Error profile
    msg.error_speed = err[0] # (m/s)
    msg.error_lateral = err[1] # (m)
    msg.error_yaw = err[2] # (rad)
    # Actual state
    msg.actual_x = state['x'] # (m)
    msg.actual_y = state['y'] # (m)
    msg.actual_yaw = state['yaw'] # (rad)
    msg.actual_speed = state['v'] # (m/s)
    # Reference value
    msg.wp_idx = controller.get_closest_index()
    msg.ref_x = ref[0] # (m)
    msg.ref_y = ref[1] # (m)
    msg.ref_yaw = ref[2] # (rad)
    msg.ref_speed = ref[3] # (m/s)
    msg.ref_curvature = ref[4] # (1/m)
    # In degree
    msg.deg_ref_yaw = msg.ref_yaw * 180. / np.pi # degree
    msg.deg_actual_yaw = msg.actual_yaw * 180. / np.pi # degree
    msg.deg_error_yaw = msg.error_yaw * 180. / np.pi # degree
    # Publish the message
    pub.publish(msg)

    ### Wait until the next loop
    rate.sleep()
