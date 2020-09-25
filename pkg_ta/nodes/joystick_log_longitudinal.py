#!/usr/bin/env python
import rospy
import numpy as np
import pygame

from pkg_ta.msg import Control
from std_msgs.msg import Float64

lon = 0.4

# Joystick ROS
def main():
    global lon
    
    # Initiate the joystick
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)
    js.init()
    
    # Initialite node and topic
    rospy.init_node('joystick')
    pub=rospy.Publisher('/control_signal', Control, queue_size=1)
    rate= rospy.Rate(20) # Hz
    
    # Create the msg object
    msg = Control()
    msg.header.frame_id = 'joystick_control'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # Main code
        pygame.event.get()
        jsInputs = [float(js.get_axis(i)) for i in range(js.get_numaxes())]

        if (-jsInputs[1]) >= 0.5:
            msg.action_throttle = lon
        else:
            msg.action_throttle = 0.0
            
        # Send the message
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
