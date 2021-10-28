from __future__ import print_function
from dronekit import connect, VehicleMode
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
	print(" Mode: %s" % vehicle.mode.name)
	time.sleep(1)

