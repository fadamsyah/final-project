#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Keyboard controlling for CARLA. Please refer to client_example.py for a simpler
# and more documented example.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot

    R            : restart level

STARTING in a moment...
"""

from __future__ import print_function

import argparse
import logging
import random
import time
import sys
import os
import pandas as pd
import pathlib

import pygame
from pygame.locals import K_DOWN
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w

import numpy as np

sys.path.append(os.path.abspath(sys.path[0] + '/..'))
from carla import image_converter
from carla import sensor
from carla.client import make_carla_client, VehicleControl
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

WINDOW_WIDTH = 200
WINDOW_HEIGHT = 150
MINI_WINDOW_WIDTH = 80
MINI_WINDOW_HEIGHT = 45

############################### MODIFICATION #################################################################################
import math
from Controller.Controller_2D import Controller_v1

csv_header = "step,rate,time,speed,throttle,brake,steer,x,y,yaw,idx,ev,elat,eyaw,sp_x,sp_y,sp_yaw,sp_v,sp_curvature"

filenum = 0
filename = "./data_logging/data" + str(filenum) + ".csv"
while pathlib.Path(filename).is_file():
    filenum += 1
    filename = "./data_logging/data" + str(filenum) + ".csv"
##############################################################################################################################


def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()
    settings.set(
        SynchronousMode=False,
        SendNonPlayerAgentsInfo=False,
        NumberOfVehicles=0,
        NumberOfPedestrians=0,
        SeedVehicles=0,
        SeedPedestrians=0,
        #PlayerVehicle="/Game/Blueprints/Vehicles/DodgeChargePolice/DodgeChargePolice.DodgeChargePolice_C",
        WeatherId=9,
        QualityLevel=args.quality_level)
    #settings.randomize_seeds()
    if args.lidar:
        lidar = sensor.Lidar('Lidar32')
        lidar.set_position(0, 0, 2.5)
        lidar.set_rotation(0, 0, 0)
        lidar.set(
            Channels=32,
            Range=50,
            PointsPerSecond=100000,
            RotationFrequency=10,
            UpperFovLimit=10,
            LowerFovLimit=-30)
        settings.add_sensor(lidar)
    return settings


class Timer(object):
    def __init__(self):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()

    def tick(self):
        self.step += 1

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) / self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time


class CarlaGame(object):
    def __init__(self, carla_client, args):
        self.client = carla_client
        self._carla_settings = make_carla_settings(args)
        self._timer = None
        self._display = None
        self._main_image = None
        self._mini_view_image1 = None
        self._mini_view_image2 = None
        self._enable_autopilot = args.autopilot
        self._lidar_measurement = None
        self._map_view = None
        self._is_on_reverse = False
        self._display_map = args.map
        self._city_name = None
        self._map = None
        self._map_shape = None
        self._map_view = None
        self._position = None
        self._agent_positions = None

        ############################### MODIFICATION #################################################################################
        self._log = args.log
        self._last_game_time = 0
        self._last_speed = 0

        waypoints_np = np.load('waypoints_interpolated.npy')
        waypoints_np[:, -1] = 0.
        self._conv_rad_to_steer = 180.0 / 70.0 / np.pi

        # Lamda 1 | 15 [0.41588377 0.44902458 0.05147478]
        self._controller = Controller_v1(0.41588377, 0.44902458, 0.05147478,\
                            np.array([0.96, 0.13, 0.15]), np.array([-1., 1.]),\
                            2.5, 1.0, 2.5, 0.01, np.array([-1.22, 1.22]),
                            waypoints_np)

        self._idx = 0

        if self._log:
            print("Logging Enabled")
            self.data_logging = open(filename, "a+")
            self.data_logging.write(csv_header + '\r\n')
        ###########################################################################################################################

    def execute(self):
        """Launch the PyGame."""
        pygame.init()
        self._initialize_game()
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                self._on_loop()
        finally:
            pygame.quit()

    def _initialize_game(self):
        self._on_new_episode()

        if self._city_name is not None:
            self._map = CarlaMap(self._city_name, 0.1643, 50.0)
            self._map_shape = self._map.map_image.shape
            self._map_view = self._map.get_map(WINDOW_HEIGHT)

            extra_width = int((WINDOW_HEIGHT/float(self._map_shape[0]))*self._map_shape[1])
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH + extra_width, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
        else:
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

        logging.debug('pygame started')

    def _on_new_episode(self):
        #self._carla_settings.randomize_seeds()
        #self._carla_settings.randomize_weather()
        scene = self.client.load_settings(self._carla_settings)
        if self._display_map:
            self._city_name = scene.map_name
        number_of_player_starts = len(scene.player_start_spots)
        player_start = np.random.randint(number_of_player_starts)
        print('Starting new episode...')
        self.client.start_episode(player_start)
        self._timer = Timer()
        self._is_on_reverse = False

    def _on_loop(self):
        self._timer.tick()

        measurements, sensor_data = self.client.read_data()

        self._lidar_measurement = sensor_data.get('Lidar32', None)

        # Print measurements every second.
        if self._timer.elapsed_seconds_since_lap() > 1.0:
            if self._city_name is not None:
                # Function to get car position on map.
                map_position = self._map.convert_to_pixel([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])
                # Function to get orientation of the road car is in.
                lane_orientation = self._map.get_lane_orientation([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])

                self._print_player_measurements_map(
                    measurements.player_measurements,
                    map_position,
                    lane_orientation)
            else:
                #self._print_player_measurements(measurements.player_measurements)
                None

            # Plot position on the map as well.

            self._timer.lap()

        control = self._final_project(measurements.player_measurements, \
                                        measurements)
        # Set the player position
        if self._city_name is not None:
            self._position = self._map.convert_to_pixel([
                measurements.player_measurements.transform.location.x,
                measurements.player_measurements.transform.location.y,
                measurements.player_measurements.transform.location.z])
            self._agent_positions = measurements.non_player_agents

        if control is None:
            self._on_new_episode()
        elif self._enable_autopilot:
            self.client.send_control(measurements.player_measurements.autopilot_control)
        else:
            self.client.send_control(control)

    ############################### MODIFICATION #################################################################################
    def _final_project(self, player_measurements, measurements):
        now = float(measurements.game_timestamp) / 1000.0 # second
        step = self._timer.step
        delta_t = now - self._last_game_time
        self._last_game_time = now

        speed = float(player_measurements.forward_speed) # m/s
        x = float(player_measurements.transform.location.x)
        y = float(player_measurements.transform.location.y)
        yaw = math.radians(player_measurements.transform.rotation.yaw)

        control = VehicleControl()
        control.steer = 0
        control.throttle = 0
        control.brake = 0
        control.hand_brake = False
        control.reverse = False

        start_time_threshold = 2
        if now > start_time_threshold:
            u, control.steer = self._controller.calculate_control_signal(delta_t, x, y, speed, yaw)
            control.steer = control.steer * self._conv_rad_to_steer
            control.throttle = max(u, 0.)
            control.brake = max(-u, 0.)
            trajectory = self._controller.get_instantaneous_setpoint()
            self._idx += 1
            if self._log:
                message = '{step},{rate},{time},{speed},{throttle},{brake},{steer},{x},{y},{yaw},{idx},{ev},{elat},{eyaw},{sp_x},{sp_y},{sp_yaw},{sp_v},{sp_curvature}'
                message = message.format(
                    step=step,
                    rate=delta_t,
                    time=now - start_time_threshold,
                    speed=speed,
                    throttle=control.throttle,
                    brake=control.brake,
                    steer=control.steer,
                    x=x,
                    y=y,
                    yaw=yaw,
                    idx=self._controller._closest_idx,
                    ev=self._controller._ev,
                    elat=self._controller._e_lat,
                    eyaw=self._controller._e_yaw,
                    sp_x=trajectory[0],
                    sp_y=trajectory[1],
                    sp_yaw=trajectory[2],
                    sp_v=trajectory[3],
                    sp_curvature=trajectory[4],
                )
                self.data_logging.write(message + '\r\n')

        self._last_speed = speed
        if step % 50 == 0:
            print('now: {:.2f} || step: {} || rate: {:.3f} s|| speed: {:.3f} m/s || steer: {:.3f} || throttle: {:.3f} || brake: {:.3f}' \
                .format(now, step, delta_t, speed, control.steer, control.throttle, control.brake))

        return control
    #########################################################################################################################################

    def _print_player_measurements_map(
            self,
            player_measurements,
            map_position,
            lane_orientation):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += 'Map Position ({map_x:.1f},{map_y:.1f}) '
        message += 'Lane Orientation ({ori_x:.1f},{ori_y:.1f}) '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            map_x=map_position[0],
            map_y=map_position[1],
            ori_x=lane_orientation[0],
            ori_y=lane_orientation[1],
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed * 3.6,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)

    def _print_player_measurements(self, player_measurements):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed * 3.6,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--lidar',
        action='store_true',
        help='enable Lidar')
    argparser.add_argument(
        '-log', '-log',
        action='store_true',
        help='Enable Logging')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Low',
        help='graphics quality level, a lower level makes the simulation run considerably faster')
    argparser.add_argument(
        '-m', '--map',
        action='store_true',
        help='plot the map of the current city')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    while True:
        try:

            with make_carla_client(args.host, args.port) as client:
                game = CarlaGame(client, args)
                game.execute()
                break

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
