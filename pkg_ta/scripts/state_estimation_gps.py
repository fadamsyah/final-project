import numpy as np
import rospy
import pymap3d as pm
from lib_ta_py.kf_gps import KF_gps
from sensor_msgs.msg import NavSatFix
from pkg_ta.msg import State

freq = 100 # Hz

# Reference point
lat0, lon0, h0 = -6.8712, 107.5738, 768

# Make the kf class
var_gps_pos =  0.5 **2
var_gps_speed = .75 **2
var_gps_yaw = 0.25 **2
var_gps_w = 0.1 **2

Q = np.eye(8)
Q[:2,:2] = np.eye(2) * 3.**2
Q[2:4,2:4] = np.eye(2) * 3.**2
Q[4:6,4:6] = np.eye(2) * 0.1**2
Q[6,6] = 1.**2
Q[7,7] = 0.1**2

x0 = np.array([0., 0.])
v0 = np.array([0., 0.])
a0 = np.array([0., 0.])
yaw0 = 0.
w0 = 0.
P0 = np.eye(8) * 1.

kf = KF_gps(var_gps_pos, var_gps_speed, var_gps_yaw, var_gps_w, Q,
            x0, v0, a0, yaw0, w0, P0)

gps_pos_last = np.array([0., 0.])
gps_t_last = 0.0
temp_pos_yaw = np.array([0., 0.])

RUN = False # Tunggu sampai data GPS masuk

def main():
    # Create the callback function
    def callback(data):
        global gps_pos_last
        global gps_t_last
        global temp_pos_yaw
        global RUN
        
        gps_pos_now = pm.geodetic2enu(data.latitude,
                                      data.longitude,
                                      data.altitude, lat0, lon0, h0)
        gps_pos_now = np.copy(gps_pos_now[:2]) * (-1) # Sumbu z dibuang
        gps_t_now = data.header.stamp.to_sec()
        
        if RUN:
            # Correct Position
            _ = kf.correct_position(gps_pos_now)

            # Correct Velocity
            dt_gps = gps_t_now - gps_t_last
            gps_vel = (gps_pos_now - gps_pos_last) / dt_gps
            _ = kf.correct_velocity(gps_vel)

            # Correct Yaw and Omega
            if np.linalg.norm(gps_vel) <= 2e-1: # Treshold-nya perlu dituning
                # Kalau mobil diam, berarti ga ada perubahan yaw
                _ = kf.correct_w(0.0)
            else:
                dpos = kf.x - temp_pos_yaw
                gps_yaw = np.arctan2(dpos[1], dpos[0])
                _ = kf.correct_yaw(gps_yaw)
        else:
            kf.x = np.copy(gps_pos_now)
            RUN = True
            
        gps_pos_last = np.copy(gps_pos_now)
        gps_t_last = np.copy(gps_t_now)
        temp_pos_yaw = np.copy(kf.x)

    rospy.init_node('gps_state_estimation')
    rospy.Subscriber('/fix', NavSatFix, callback)
    pub = rospy.Publisher('/gps_state_estimation', State, queue_size=1)
    rate = rospy.Rate(freq) # Hz
          
    print("Menunggu data GPS masuk pertama kali !")
    while not RUN:
        pass # INI DI WHILE KALAU DATA GPS BELUM MASUK BUAT INISIALISASI
    print("Data GPS sudah masuk !")
    print("Program sudah berjalan !")
        
    msg = State()
    msg.header.frame_id = 'gps_state_estimation_local_frame'
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    last_time = msg.header.stamp.to_sec() - 1./freq

    while not rospy.is_shutdown():
        # Calculate the actual sampling time
        msg.header.stamp = rospy.Time.now()
        delta_t = msg.header.stamp.to_sec() - last_time
        last_time = msg.header.stamp.to_sec()
        
        _ = kf.predict(delta_t)
        
        # Send the message
        msg.header.seq += 1
        msg.x, msg.y = kf.x
        msg.vx, msg.vy = kf.v
        msg.ax, msg.ay = kf.a
        msg.yaw = kf.yaw
        msg.w = kf.w

        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
