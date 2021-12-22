from __future__ import print_function
from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from math import *
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

def distance(lat1, lat2, lon1, lon2):
     
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a))
    
    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371
      
    # calculate the result
    return(c * r)

def get_current_location():
    print(vehicle.location.global_relative_frame)

while True:
    lat1 = 13.8468948
    lon1 = 100.5662444
    alt1 = -0.051
    # lat = vehicle.location.global_relative_frame.alt
    # lon = vehicle.location.global_relative_frame.lon
    # currentLocation = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, alt)

    targetLocation = LocationGlobalRelative(lat1, lon1, alt1)
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    print("targetDistance: ", targetDistance)

    lat2 = currentLocation.lat
    lon2 = currentLocation.lon
    d = distance(lat1, lat2, lon1, lon2)
    print("distance: ", d*1000)
    print(" ")
    # print("targetDistance: ", targetDistance)


