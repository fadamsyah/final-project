#!/usr/bin/env python3

# By F. Adamsyah Ma'ani & Prasetyo Wibowo L. S. (2020)
# Engineering Physics, Bandung Institute of Technology

# Program untuk merata-ratakan bacaan GPS dalam rentang waktu tertentu
# Cara pakai :
# python accumulate_gps.py -t detik
# (default = 30 detik)


import os
import argparse
import numpy as np
import pymap3d as pm
import rospy
import time
from sensor_msgs.msg import NavSatFix

# Reference point
lat0, lon0, h0 = -6.8712, 107.5738, 768

freq = 28 # 2 kali freq GNSS

RUN_gps = False # Tunggu sampai data posisi masuk
state_x = []
state_y = []
time_now = time.time()

def main():
    parser = argparse.ArgumentParser(description='Average GPS data within certain time')
    parser.add_argument('-t', type=int, default=30,
                        help='Rentang waktu akumulasi data GPS (detik)')

    args = parser.parse_args()

    def callback_gps(msg_gps):
        global RUN_gps
        global state_x
        global state_y
        global time_now

        if not RUN_gps:
            RUN_gps = True
            time_now = time.time()
        else:
            gps_pos = pm.geodetic2enu(msg_gps.latitude,
                                      msg_gps.longitude,
                                      msg_gps.altitude, lat0, lon0, h0)
            state_x.append(gps_pos[0])
            state_y.append(gps_pos[1])

    rospy.init_node('gps_avg')
    rospy.Subscriber('/fix', NavSatFix, callback_gps)

    print("Menunggu data gps masuk pertama kali ...")
    while not RUN_gps:
        pass # INI DI WHILE KALAU DATA GPS BELUM MASUK BUAT INISIALISASI
    print("Data Navigasi sudah masuk !")
    print("Accumulating data for", args.t, "seconds...")

    r = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if time.time() - time_now > args.t:
            print("x = ", np.average(np.array(state_x)))
            print("y = ", np.average(np.array(state_y)))
            os._exit(0)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
