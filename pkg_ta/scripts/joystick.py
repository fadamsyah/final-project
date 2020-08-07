#!/usr/bin/env python
import rospy
import pygame

from pkg_ta.msg import Control

pygame.init()
pygame.joystick.init()

max_steer = 28
min_steer = -35
max_brake = 2.

# Joystick ROS
def main():
    # Initialite node and topic
    rospy.init_node('joystick')
    pub=rospy.Publisher('control_signal', Control, queue_size=1)
    rate= rospy.Rate(20) # Hz

    js = pygame.joystick.Joystick(0)
    js.init()

    while not rospy.is_shutdown():

        # Main code
        pygame.event.get()
        jsInputs = [float(js.get_axis(i)) for i in range(js.get_numaxes())]
        jsteer = jsInputs[3]
        if jsteer > 0:
            jsteer = jsteer * max_steer
        else:
            jsteer = jsteer * min_steer
        jt = -jsInputs[1]

        if abs(jsteer) < 0.001:
            jsteer = 0
        if abs(jt) < 0.001:
            jt = 0

        # Send the message
        msg = Control()
        msg.steer = jsteer
        msg.throttle = max(0, jt)
        msg.brake = max(0, -jt*max_brake)
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
