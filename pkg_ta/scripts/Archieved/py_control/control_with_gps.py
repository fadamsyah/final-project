import numpy as np
import pandas as pd
import numba as nb
from lib_ta_py.controller_2D import Controller_v1
import rospy
from pkg_ta.msg import Control
from pkg_ta.msg import State

freq = 10 # Hz
waypoints_np = np.load('waypoints/WAYPOINTS_I_4.npy')

# In the Arduino, CW is positive and CCW is negative
# On the other hand, in the controller algoritm, CCW is positive and CW is negative
max_steer = 35.; min_steer = -28. # For the path following control algoritm ~ degree
max_steer_arduino = 28.; min_steer_arduino = -35. # For the Arduino ~ degree
max_brake = 2.9; max_throttle = 0.08

kp = 0.15; ki = 0.075; kd = 0.0
ff_long = np.array([0.0, 0.0]) # no feed-forward
sat_long = np.array([-np.abs(max_brake), np.abs(max_throttle)])
sat_lat = np.array([-np.abs(min_steer), np.abs(max_steer)])
sat_lat = sat_lat * np.pi / 180.

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}

RUN = False # Tunggu sampai data lokalisasi masuk

def main():
    # Create the controller object
    controller = Controller_v1(kp, ki, kd, ff_long, sat_long,
                           0.4, 1.5, 2.4, 0.01, sat_lat,
                           waypoints_np)

    # Create the callback function
    def callback(data):
        global state
        global RUN
        
        state['x'] = data.x
        state['y'] = data.y
        state['yaw'] = data.yaw
        state['v'] = np.sqrt(data.vx**2 + data.vy**2) # m/s

        RUN = True

    rospy.init_node('control')
    rospy.Subscriber('/gps_state_estimation', State, callback)
    pub = rospy.Publisher('/control_signal', Control, queue_size=1)
    rate = rospy.Rate(freq) # Hz
            
    print("Menunggu data Navigasi masuk pertama kali !")
    while not RUN:
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
        
        '''
        print('steering_control: {}'.format(lat*180/np.pi))
        print('contrib_feedf: {}'.format(a*180/np.pi)) 
        print('contrib_e_yaw: {}'.format(b*180/np.pi))
        print('contrib_e_lat: {}'.format(c*180/np.pi))
        print('ddddddddddddd: {}'.format(d*180/np.pi))
        print('eeeeeeeeeeeee: {}'.format(e*180/np.pi))
        print('fffffffffffff: {}'.format(f*180/np.pi))
        print('ggggggggggggg: {}'.format(g*180/np.pi))
        print('hhhhhhhhhhhhh: {}'.format(h*180/np.pi))
        print('iiiiiiiiiiiii: {}'.format(i*180/np.pi))
        '''

        # Get the error profile
        err = controller.get_error()
        # Get the reference
        ref = controller.get_instantaneous_setpoint()
        
        # Send the message
        msg.header.seq += 1
        msg.action_steer = max(min(-lat*180/np.pi, max_steer_arduino), min_steer_arduino) # lat ~ radian
        #msg.action_throttle = max(min(long, max_throttle), 0.)
        msg.action_throttle = max_throttle
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
