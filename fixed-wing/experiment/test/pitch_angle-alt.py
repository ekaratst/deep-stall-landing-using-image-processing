#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse

connection_string = "/dev/ttyACM0"
baud_rate = 57600

#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41, 26.18]
radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]
timer_exit = 10

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')



pitch_down_start_time = time.time()
while True:
    current_alttitude = vehicle.location.global_relative_frame.alt
    pitch_angle = math.degrees(vehicle.attitude.pitch) 
    pitch_down_now_time = time.time()
    print("pitch angle: ", pitch_angle)
    if pitch_down_now_time - pitch_down_start_time >= 5:
        print("time out pitch down")
        break
    if current_alttitude <= 10.5:
        print("Reached deepstall altitude.")
        break
    if pitch_angle  >= -12:
        print("Elevator down")
        vehicle.channels.overrides['2'] = 1312
    else:
        vehicle.channels.overrides['2'] = 1500
        print("Pitch angle has been adjusted to -15[deg]")
    
#deepstall
pitch_up_start_time = time.time()
while True:
    pitch_angle = math.degrees(vehicle.attitude.pitch) 
    pitch_up_now_time = time.time()
    print("pitch angle: ", pitch_angle)
    if pitch_up_now_time - pitch_up_start_time >= 5:
        print("time out pitch up")
        break
    if pitch_angle  <= 27:
        print("Elevator up")
        vehicle.channels.overrides['2'] = 1924
    else:
        vehicle.channels.overrides['2'] = 1500
        print("Pitch angle has been adjusted to 60[deg]")
        break

#if vehicle.mode.name=='STABILIZE':
# pitch_down_start_time = time.time()
# while True:
#     pitch_angle = math.degrees(vehicle.attitude.pitch) 
#     pitch_down_now_time = time.time()
#     print("pitch angle: ", pitch_angle)
#     if pitch_down_now_time - pitch_down_start_time >= 7:
#         print("time out pitch down")
#         break
#     if pitch_angle  >= -12:
#         print("Elevator down")
#         vehicle.channels.overrides['2'] = 1924
#     else:
#         vehicle.channels.overrides['2'] = 1485
#         print("Pitch angle has been adjusted to -15[deg]")
#         break
#     time.sleep(0.5)
# time.sleep(1)
# pitch_up_start_time = time.time()
# while True:
#     pitch_angle = math.degrees(vehicle.attitude.pitch) 
#     pitch_up_now_time = time.time()
#     print("pitch angle: ", pitch_angle)
#     if pitch_up_now_time - pitch_up_start_time >= 7:
#         print("time out pitch up")
#         break
#     if pitch_angle  <= 27:
#         print("Elevator up")
#         vehicle.channels.overrides['2'] = 1104
#     else:
#         vehicle.channels.overrides['2'] = 1485
#         print("Pitch angle has been adjusted to 60[deg]")
#         break
#     time.sleep(0.5)
# Doing adjust elevator
    
