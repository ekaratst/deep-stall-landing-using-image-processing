#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse
import time
import xlsxwriter

timestr = time.strftime("%Y-%m-%d_%H-%M-%S")
print(timestr)

workbook = xlsxwriter.Workbook("log/" + timestr + "_log.xlsx")
worksheet = workbook.add_worksheet("My sheet")

row = 0
col = 0

worksheet.write(row, col, "Altitude")
worksheet.write(row, col + 1, "Horizontal distance")

n = 0

row += 1
ratio_time = 0
connection_string = "/dev/ttyACM0"
baud_rate = 57600

#-------------(deg) 3      0     -5     -10    -15   -20
simulate_angle = [52.5, 49.82, 45.51, 40.33, 35.11, 28.86]
ch2in = [1472, 1525, 1639, 1741, 1870, 2028]
delta_angle = [3, 0, -5, -10, -15, -20]

# #1678 -> 10 deg
# #1507 -> Neutral
# #1186 -> -30 deg
# #-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
# simulate_angle = [52.5, 49.82, 45.51, 40.33, 35.11, 28.86, 23.81]
# radio_in_elevator = [1467, 1478, 1602, 1683, 1764, 1838, 1924] #ch2in radio calibration
# delta_angle = [3, 0, -5, -10, -15, -20, -25]



print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

isnot_deepstalled = True
is_deepstalled = False

def deepstall(is_deepstalled,row, col, ratio_time):
	# print("Thread-2")
	# start = time.time()
	lat = 13.8471013
	lon = 100.5658087
	alt = 0
	target_waypoint_location = LocationGlobalRelative(lat,lon,alt)
	# target_waypoint_location = vehicle.location.global_relative_frame
	while True:
		altitude_now = vehicle.location.global_relative_frame.alt
		print("Alt: ", altitude_now)
		current_waypoint_location = vehicle.location.global_relative_frame
		print("ch7: ", vehicle.channels['7']) # G switch
		print("distance: ", get_distance_metres(current_waypoint_location, target_waypoint_location))
		if int(vehicle.channels['7']) > 1514 and not is_deepstalled: # toggle when enter auto mode
			if get_distance_metres(current_waypoint_location, target_waypoint_location) <= 9:
				poststall_waypoint_location = vehicle.location.global_relative_frame
				print(is_deepstalled)
				vehicle.mode = VehicleMode("STABILIZE")
				vehicle.channels.overrides['2'] = 2033
				# if n_deepstall == 0:
				start_time = time.time()
				is_deepstalled = True
				print("Deep stall!!!")	

		if int(vehicle.channels['7']) < 1514:
			is_deepstalled = False
			vehicle.channels.overrides = {}
			print("Pilot control")
		
		if int(vehicle.channels['7']) > 1514 and is_deepstalled:
			# post_stall(start_time, row ,col, ratio_time)
			vehicle.channels.overrides['2'] = 2033
			post_stall_time = time.time()
			print ("Groundspeed: %s" % vehicle.groundspeed)
			diff_time = float(post_stall_time) - float(start_time)
			if diff_time >= 0.1 * ratio_time:
				print(diff_time)
				current_altitude = vehicle.location.global_relative_frame.alt
				# horizontal_distance  = 0.1 * vehicle.groundspeed
				horizontal_distance = get_distance_metres(current_waypoint_location, poststall_waypoint_location)
				worksheet.write(row, col, current_altitude)
				worksheet.write(row, col + 1, horizontal_distance)
				row += 1
				ratio_time += 1
			if current_altitude <= 1 and int(vehicle.channels['7']) > 1514:
				workbook.close()
				print("end log!!")
				
		print("-------------------------------------")
		# time.sleep(1)

def adjustElevator(trajectory_angle, is_deepstalled):
	if vehicle.mode.name == "AUTO" and is_deepstalled:
	# if int(vehicle.channels['7']) > 1514:
		if trajectory_angle >= simulate_angle[0]:
			vehicle.channels.overrides['2'] = ch2in[0]
			print("adjust elevator angle to: 3 deg")
		elif trajectory_angle >= simulate_angle[1]:
			vehicle.channels.overrides['2'] = ch2in[1]
			print("adjust elevator angle to: 0 deg")
		elif trajectory_angle >= simulate_angle[2]:
			vehicle.channels.overrides['2'] = ch2in[2]
			print("adjust elevator angle to: -5 deg")
		elif trajectory_angle >= simulate_angle[3]:
			vehicle.channels.overrides['2'] = ch2in[3]
			print("adjust elevator angle to: -10 deg")
		elif trajectory_angle >= simulate_angle[4]:
			vehicle.channels.overrides['2'] = ch2in[4]
			print("adjust elevator angle to: -15 deg")
		else:
			vehicle.channels.overrides['2'] = ch2in[5]
			print("adjust elevator angle to: -20 deg")
		# else:
		# 	vehicle.channels.overrides['2'] = 1911
		# 	print("adjust elevator angle to: -25 deg")

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

try:				
	_thread.start_new_thread( deepstall, (is_deepstalled, row, col, ratio_time, ))
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
