import rospy
import numpy as np
import pygame
from pkg_ta.msg import Control, LogBrakeCommand

pwm = 0
setpoint_brake = 0.

def main():
    # Define the saturation value
    pwm_max = 255
    pos_max = 3.
    
    # Initiate the joystick
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)
    js.init()
    
    # Make the callback of the subscriber
    def callback(sub_msg):
        global pwm, setpoint_brake
        pwm = sub_msg.pwm
        setpoint_brake = sub_msg.action_brake 

    # Initialize node and topic
    rospy.init_node('log_brake')
    rospy.Subscriber('brake_command', LogBrakeCommand, callback)
    pub = rospy.Publisher('control_signal', LogBrakeCommand, queue_size=1)
    rate = rospy.Rate(20) # Hz
    
    # Create the msg objecct
    msg = LogBrakeCommand()
    msg.header.frame_id = 'log_brake'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.pwm = 0
    msg.action_brake = 0.
    
    while not rospy.is_shutdown():
        # Main code
        pygame.event.get()
        jsInputs = [float(js.get_axis(i)) for i in range(js.get_numaxes())]
        
        if (-jsInputs[1]) >= 0.9:
            msg.pwm = max(min(pwm, pwm_max), 0)
            msg.action_brake = max(min(setpoint_brake, pos_max), 0.)
        else:
            msg.pwm = 0
            msg.action_brake = 0.
            
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
