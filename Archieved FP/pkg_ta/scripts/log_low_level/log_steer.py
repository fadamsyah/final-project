#!/usr/bin/env python
import rospy
import numpy as np

from pkg_ta.msg import Control

def main():  
    # Define the saturation value of the actuator, i.e. steer, brake, and throttle
    max_steer = 28.
    min_steer = -35.
    
    # Initialite node and topic
    rospy.init_node('log_steer')
    pub=rospy.Publisher('control_signal', Control, queue_size=1)
    rate=rospy.Rate(20) # Hz
    
    # Create the msg object
    msg = Control()
    msg.header.frame_id = 'log_steer'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.action_throttle = 0.0
    msg.action_brake = 0.0
    msg.action_steer = 0.0

    while not rospy.is_shutdown():
        # Main code
        if msg.header.seq > 800:
            msg.action_steer = 0.
        elif msg.header.seq > 550:
            msg.action_steer = min_steer
        elif msg.header.seq > 300:
            msg.action_steer = max_steer
        elif msg.header.seq > 150:
            msg.action_steer = min_steer
        # Send the message
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
