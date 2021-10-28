from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    print(" Velocity: %s" % vehicle.velocity)   
    print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    print(" Airspeed: %s" % vehicle.airspeed)    # settable
    print("Current Altitude: %s" %  current_altitude)
    
