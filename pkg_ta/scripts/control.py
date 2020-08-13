import numpy as np
import pandas as pd
import ...
import rospy
from pkg_ta.msg import Control

max_steer = 28.; min_steer = -35.
max_brake = 2.; max_throttle = 1.

kp = ...; ki = ...; kd = ...
ff_long = np.array([..., ...])
sat_long = np.array([-max_brake, max_throttle])
sat_lat = np.array([min_steer, max_steer])



state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}


def main():
    controller = Controller_v1(kp, ki, kd, ff_long, sat_long,
                           2.5, 1.0, 2.5, 0.01, sat_lat,
                           waypoints_np)

    def callback(data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        global state
        state['x'] = ...
        state['y'] = ...
        state['yaw'] = ...
        state['v'] = ...

    rospy.init_node('control')
    rospy.Subscriber("...", Odometry, callback)
    pub = rospy.Publisher('control_signal', Control, queue_size=1)
    rate = rospy.Rate(20) # Hz
            
    msg = Control()

    while not rospy.is_shutdown():
        # delta_t belum dihitung .....
        long, lat = controller.calculate_control_signal(delta_t, state['x'],
                                                        state['y'], state['v'],
                                                        state['yaw'])
        # Send the message
        msg.action_steer = max(min(lat, max_steer), min_steer)
        msg.action_throttle = max(min(long, max_throttle), 0.)
        msg.action_brake = max(min(-long, max_brake), 0.)
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
