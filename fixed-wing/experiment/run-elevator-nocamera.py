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

def deepstall(is_deepstalled):
	# print("Thread-2")
	# start = time.time()
	# lat = 13.8472679
	# lon = 100.5659160
	# alt = 10
	# targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	while True:
		# if int(vehicle.channels['7']) > 1514:
		# 	vehicle.channels.overrides = {}
		# 	is_deepstalled = False

		# lat = vehicle.location.global_relative_frame.alt
		# lon = vehicle.location.global_relative_frame.lon
		# print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
		# print("Deep stall using Altitude")
		
		# print("alttitude: ", alttitude)
		vehicle.mode = VehicleMode("STABILIZE")
		print("Mode:", vehicle.mode.name)
		# if vehicle.location.global_relative_frame.lat <= 

		# if int(vehicle.channels['6']) < 1514: # D switch
		# 	print("Deep stall using STABILIZE Mode")
		# 	if vehicle.mode.name == "STABILIZE":
		# 		print("ch7: ", vehicle.channels['7']) # G switch
		# 		if int(vehicle.channels['7']) > 1514: # toggle when enter auto mode
		# 			if int(vehicle.channels['6']) < 1514:
		# 				pitch_angle = math.degrees(vehicle.attitude.pitch) 
		# 				print("pitch_angle: ", pitch_angle)
		# 			# if pitch_angle  >= 60 :
		# 			# if get_distance_metres(vehicle.location.global_relative_frame, targetWaypointLocation) <= 3:
		# 				vehicle.channels.overrides['2'] = 2114
		# 				print("Deep stall!!!")
		# 				is_deepstalled = True
		# 		else:
		# 			is_deepstalled = False
		# 			print("Pilot control")
		# 			vehicle.channels.overrides = {}
		# else:
		start_time = toggle_deepstall(is_deepstalled)
		post_stall(start_time, row ,col)

		
		# if is_deepstalled == True:
		# 	vehicle.channels.overrides['2'] = 1925
		# 	print("Elevator up")
		# else:
		# 	vehicle.channels.overrides = {}
		print("-------------------------------------\n")

def toggle_deepstall(is_deepstalled):
	print("ch7: ", vehicle.channels['7']) # G switch
	if int(vehicle.channels['7']) > 1514 and not is_deepstalled: # toggle when enter auto mode
		vehicle.mode = VehicleMode("STABILIZE")
		vehicle.channels.overrides['2'] = 1925
		start_time = time.time()
		is_deepstalled = True
		print("Deep stall!!!")	
		return start_time
	else:
		is_deepstalled = False
		vehicle.channels.overrides = {}
		print("Pilot control")

def post_stall(start_time, row, col):
	vehicle.channels.overrides['2'] = 1925
	post_stall_time = time.time()
	print ("Groundspeed: %s" % vehicle.groundspeed)
	del_time = post_stall_time - start_time
	print(del_time)
	# if (post_stall_time - start_time) % 0.1 == 0:
	# 	altitude = vehicle.location.global_relative_frame.alt
	# 	x_distance  = 0.1 * vehicle.groundspeed
	# 	worksheet.write(row, col, altitude)
	# 	worksheet.write(row, col + 1, x_distance)
	# 	row += 1
	time.sleep(0.5)


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
	# deepstall(is_deepstalled)
	_thread.start_new_thread( deepstall, (is_deepstalled, ))
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
