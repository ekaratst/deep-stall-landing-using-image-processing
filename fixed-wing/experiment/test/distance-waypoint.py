from __future__ import print_function
from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

lat = 13.8468406
lon = 100.566214
alt = -0.038
targetLocation = LocationGlobalRelative(lat,lon,alt)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

while True:
    # lat = vehicle.location.global_relative_frame.alt
    # lon = vehicle.location.global_relative_frame.lon
    print(vehicle.location.global_relative_frame)
    # currentLocation = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, alt)
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    print("targetDistance: ", targetDistance)
    altitude = vehicle.location.global_relative_frame.alt

