#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse
import time
import xlsxwriter
import logging

timestr = time.strftime("%Y-%m-%d_%H-%M-%S")
print(timestr)

workbook = xlsxwriter.Workbook("log/" + timestr + "_log.xlsx")
worksheet = workbook.add_worksheet("My sheet")

logging.basicConfig(filename='flight record/'+ timestr + '_flight_record_.log',  level=logging.INFO, format='%(asctime)s %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
print("Created flight record.")

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

timer_exit = 10

#--- Define Tag
id_to_find  = 72
marker_size  = 50 #- [cm]

font = cv2.FONT_HERSHEY_PLAIN

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()



isnot_deepstalled = True
is_deepstalled = False
angle_to_be_adjusted = 1712

def deepstall(is_deepstalled,row, col, ratio_time, angle_to_be_adjusted):
	# printfr("Thread-2")
	# start = time.time()
	lat = 13.8471013
	lon = 100.5658087
	alt = 0
	target_waypoint_location = LocationGlobalRelative(lat,lon,alt)
	# target_waypoint_location = vehicle.location.global_relative_frame
	while True:
		altitude_now = vehicle.location.global_relative_frame.alt
		printfr("-------------Check stall condition-------------")
		printfr("Alt: " + str(altitude_now))
		# printfr("ch8: " + str(vehicle.channels['8'])) # G switch
		current_waypoint_location = vehicle.location.global_relative_frame
		printfr("distance: " + str(get_distance_metres(current_waypoint_location, target_waypoint_location)))
		printfr("Mode: " + str(vehicle.mode.name))
		# printfr("-------------------------------------")

		# -- Deep stall conditions
		# if int(vehicle.channels['8']) > 1514 and not is_deepstalled: # toggle when enter auto mode
		if int(vehicle.channels['8']) < 1514:
			is_deepstalled = False
			vehicle.channels.overrides = {}
			printfr("Pilot control")

		if vehicle.mode.name == 'AUTO' and not is_deepstalled and int(vehicle.channels['8']) > 1514:
			printfr("Changed mode to: " + str(vehicle.mode.name))
			printfr("Waiting for reach target...")
			if get_distance_metres(current_waypoint_location, target_waypoint_location) <= 25: #9, 25
				poststall_waypoint_location = vehicle.location.global_relative_frame
				printfr("is_deepstalled: " + str(is_deepstalled))
				vehicle.mode = VehicleMode("STABILIZE")
				vehicle.channels.overrides['2'] = 1800
				# if n_deepstall == 0:
				start_time = time.time()
				is_deepstalled = True
				printfr("-------------Deep stall!!!-------------")	

		# -- Not stall
		# if int(vehicle.channels['8']) < 1514:
		if vehicle.mode.name != 'AUTO' and int(vehicle.channels['8']) < 1514:
			is_deepstalled = False
			vehicle.channels.overrides = {}
			printfr("Pilot control")
		
		# -- Post stall
		current_altitude = vehicle.location.global_relative_frame.alt
		# if int(vehicle.channels['8']) > 1514 and is_deepstalled and current_altitude > 10:
		if int(vehicle.channels['8']) > 1514 and is_deepstalled:
			if current_altitude > 10: #10
				printfr("-------------Post stall-------------")
				# post_stall(start_time, row ,col, ratio_time)
				vehicle.channels.overrides['2'] = 1800
				

			# -- Adjust elevator angle
			else:
				printfr("-------------Adjust angle after stall-------------")
				printfr(str(angle_to_be_adjusted))
				vehicle.channels.overrides['2'] = angle_to_be_adjusted
			
			post_stall_time = time.time()
			printfr("Groundspeed: " + str(vehicle.groundspeed))
			diff_time = float(post_stall_time) - float(start_time)
			if diff_time >= 0.1 * ratio_time:
				printfr("excel log")
				# printfr(diff_time)
				current_altitude = vehicle.location.global_relative_frame.alt
				# horizontal_distance  = 0.1 * vehicle.groundspeed
				horizontal_distance = get_distance_metres(current_waypoint_location, poststall_waypoint_location)
				worksheet.write(row, col, current_altitude)
				worksheet.write(row, col + 1, horizontal_distance)
				row += 1
				ratio_time += 1
			if current_altitude <= 1:
				workbook.close()
				printfr("end log!!")
				
		# printfr("-------------------------------------")
		# time.sleep(1)
		#-- detect and adjust
			
		

def printfr(str):
	print(str)
	logging.info(str)

def adjust_elevator(trajectory_angle, is_deepstalled):
	if vehicle.mode.name == "AUTO" and is_deepstalled:
	# if int(vehicle.channels['8']) > 1514:
		if trajectory_angle >= simulate_angle[0]:
			vehicle.channels.overrides['2'] = ch2in[0]
			printfr("adjust elevator angle to: 3 deg")
		elif trajectory_angle >= simulate_angle[1]:
			vehicle.channels.overrides['2'] = ch2in[1]
			printfr("adjust elevator angle to: 0 deg")
		elif trajectory_angle >= simulate_angle[2]:
			vehicle.channels.overrides['2'] = ch2in[2]
			printfr("adjust elevator angle to: -5 deg")
		elif trajectory_angle >= simulate_angle[3]:
			vehicle.channels.overrides['2'] = ch2in[3]
			printfr("adjust elevator angle to: -10 deg")
		elif trajectory_angle >= simulate_angle[4]:
			vehicle.channels.overrides['2'] = ch2in[4]
			printfr("adjust elevator angle to: -15 deg")
		else:
			vehicle.channels.overrides['2'] = ch2in[5]
			printfr("adjust elevator angle to: -20 deg")
		# else:
		# 	vehicle.channels.overrides['2'] = 1911
		# 	printfr("adjust elevator angle to: -25 deg")

def is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotation_matrix_to_euler_angles(R):
    assert (is_rotation_matrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

try:	
	printfr('Connecting to Vehicle on: ' + str(connection_string))
	vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
	vehicle.wait_ready('autopilot_version')
	# video_filename = "../../../Videos/ground/20-12-64_ground_test_" + args.number_of_run + ".avi"
	video_filename = "../../../Videos/flight/" + timestr + ".avi"
	# workbook = xlsxwriter.Workbook("log/" + timestr + "_log.xlsx")

	# cap = cv2.VideoCapture(0)

	# if (cap.isOpened() == False):
	# 	print("Error reading video file")
		
	# frame_width = int(cap.get(3))
	# frame_height = int(cap.get(4))

	# size = (frame_width, frame_height)
	
	# out = cv2.VideoWriter(video_filename,
	# 						cv2.VideoWriter_fourcc(*'MJPG'),
	# 						10, size)	

	# printfr(timestr)		
	_thread.start_new_thread( deepstall, (is_deepstalled, row, col, ratio_time, angle_to_be_adjusted,))
 
except:
	print("Error: unable to start thread")

while 1:
	pass
