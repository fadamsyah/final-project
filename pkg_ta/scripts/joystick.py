#!/usr/bin/env python
import rospy
import numpy as np
import pygame

from pkg_ta.msg import Control

# Joystick ROS
def main():
    # Initiate the joystick
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)
    js.init()
    
    # Define the saturation value of the actuator, i.e. steer, brake, and throttle
    max_steer = 28.
    min_steer = -35.
    max_brake = 2.
    max_throttle = 1.
    
    # Initialite node and topic
    rospy.init_node('joystick')
    pub=rospy.Publisher('control_signal', Control, queue_size=1)
    rate= rospy.Rate(20) # Hz
    
    # Create the msg object
    msg = Control()
    msg.header.frame_id = 'joystick_control'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    
    long = 0.
    lat = 0.    

    while not rospy.is_shutdown():
        # Main code
        pygame.event.get()
        jsInputs = [float(js.get_axis(i)) for i in range(js.get_numaxes())]
        
        if np.abs(jsInputs[3]) < 0.01:
            jsInputs[3] = 0.
        if np.abs(jsInputs[1]) < 0.01:
            jsInputs[1] = 0.
            
        long = 0.9 * long + 0.1 * (-jsInputs[1])
        if long >= 0.:
            long = long * max_throttle
        else:
            long = long * max_brake
            
        lat = 0.9 * lat + 0.1 * jsInputs[3]
        if lat >= 0.:
            lat = lat * max_steer
        else:
            lat = lat * min_steer
            
        # Send the message
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        msg.steer = lat
        msg.throttle = max(0, long)
        msg.brake = max(0, -long)
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
