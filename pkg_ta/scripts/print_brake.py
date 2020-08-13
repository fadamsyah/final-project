import rospy
from pkg_ta.msg import LogArduino

msg = LogArduino()

# Make the callback of the subscriber
def callback(sub_msg):
    global msg
    msg = sub_msg
    
def main():
    # Initialize node and topic
    rospy.init_node('print_brake')
    rospy.Subscriber('logging_arduino', LogArduino, callback)
    
    # You need to change this
    print_period = 0.1 # 10 Hz
    last_print = rospy.get_rostime()
    last_print = last_print.secs + last_print.nsecs / 1e9 + print_period
    
    while not rospy.is_shutdown():
        # Main code
        now = rospy.get_rostime()
        now = now.secs + now.nsecs / 1e9
        if now >= last_print:
            print('setpoint: {:.3f} || position: {:.3f} || pwm: {:.3f} || R_current: {:.4f} || L_current: {:.4f} ||'.format(msg.brake_setpoint, msg.brake_position, msg.brake_pwm, msg.brake_R_current, msg.brake_L_current))
            last_print = last_print + print_period
            
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
