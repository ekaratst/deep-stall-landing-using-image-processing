#!/usr/bin/python3

from __future__ import print_function
from tracemalloc import start
from cv2 import Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse
import time
import xlsxwriter
import logging
from datetime import datetime, timedelta

INTERVAL = timedelta(seconds=3)
last_checked = datetime.now() - INTERVAL

timestr = time.strftime("%Y-%m-%d_%H-%M-%S")
print(timestr)

# workbook = xlsxwriter.Workbook("log/" + timestr + "_log.xlsx")
# worksheet = workbook.add_worksheet("My sheet")

logging.basicConfig(filename='flight record/'+ timestr + '_flight_record_.log',  level=logging.INFO, format='%(asctime)s %(message)s', datefmt='%d/%m/%Y %H:%M:%S')
print("Created flight record.")

# row = 0
# col = 0

# worksheet.write(row, col, "Altitude")
# worksheet.write(row, col + 1, "Horizontal distance")

# n = 0

# row += 1
ratio_time = 0
connection_string = "/dev/ttyACM0"
baud_rate = 57600

#-------------(deg) 0 -10 -20
simulate_angle = [1, 9, 28]
ch2in = [1515, 1712, 1880]
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
marker_size  = 100 #- [cm] #50

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

is_deepstalled = False
is_find_id = False
angle_to_be_adjusted = 1712 #1515

def deepstall():
	global is_deepstalled
	
	# printfr("Thread-2")
	# start = time.time()
	lat = 13.8472230 #13.8471013
	lon = 100.5656008 #100.5658087
	alt = 0 #0
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
		printfr("is_deepstalled(deepstall): "+ str(is_deepstalled))
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
				# start_time = time.time()
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
		printfr("angle to be adjusted: " + str(angle_to_be_adjusted))
		if int(vehicle.channels['8']) > 1514 and is_deepstalled:
			# if int(vehicle.channels['8']) > 1514 and is_deepstalled:
			if current_altitude > 10: #10
				# printfr("-------------Post stall-------------")
				# 	# post_stall(start_time, row ,col, ratio_time)
				# vehicle.channels.overrides['2'] = 1800

				printfr("-------------Adjust angle after stall-------------")
				vehicle.channels.overrides['2'] = angle_to_be_adjusted

			# -- Adjust elevator angle
			else:
				printfr("-------------Adjust angle after stall-------------")
				# printfr("angle to be adjusted: " + str(angle_to_be_adjusted))
				vehicle.channels.overrides['2'] = angle_to_be_adjusted
			
			#post_stall_time = time.time()
			#printfr("Groundspeed: " + str(vehicle.groundspeed))
			#diff_time = float(post_stall_time) - float(start_time)
			#if diff_time >= 0.1 * ratio_time:
		#		printfr("excel log")
				# printfr(diff_time)
			#	current_altitude = vehicle.location.global_relative_frame.alt
				# horizontal_distance  = 0.1 * vehicle.groundspeed
			#	horizontal_distance = get_distance_metres(current_waypoint_location, poststall_waypoint_location)
			#	worksheet.write(row, col, current_altitude)
			#	worksheet.write(row, col + 1, horizontal_distance)
			#	row += 1
			#	ratio_time += 1
			#if current_altitude <= 1:
			#	workbook.close()
			#	printfr("end log!!")
				
		# printfr("-------------------------------------")
		# time.sleep(1)
		#-- detect and adjust	

def image_processing(cap, id_to_find, out):
	is_detected = False
	while True:
		
		printfr("is_deepstalled(image_processing): "+ str(is_deepstalled))
		#-- Read the camera frame
		ret, frame = cap.read()

		#-- Convert in gray scale
		gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

		#-- Find all the aruco markers in the image
		corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
								cameraMatrix=camera_matrix, distCoeff=camera_distortion)
		
		if ids is not None and ids[0] == id_to_find:
			is_detected = True

			#-- ret = [rvec, tvec, ?]
			#-- array of rotation and position of each marker in camera frame
			#-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
			#-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
			ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

			#-- Unpack the output, get only the first
			rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

			#-- Draw the detected marker and put a reference frame over it
			aruco.drawDetectedMarkers(frame, corners)
			aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

			x_position = tvec[0]
			y_position = tvec[1]
			z_position = tvec[2]
			
			str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(x_position, y_position, z_position)
			cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#-- Obtain the rotation matrix tag->camera
			R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
			R_tc    = R_ct.T

			#-- Get the attitude in terms of euler 321 (Needs to be flipped first)
			roll_marker, pitch_marker, yaw_marker = rotation_matrix_to_euler_angles(R_flip*R_tc)

			#-- print the marker's attitude respect to camera frame
			str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
								math.degrees(yaw_marker))
			cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


			#-- Now get Position and attitude f the camera respect to the marker
			pos_camera = -R_tc*np.matrix(tvec).T

			str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
			cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#-- Get the attitude of the camera respect to the frame
			roll_camera, pitch_camera, yaw_camera = rotation_matrix_to_euler_angles(R_flip*R_tc)
			str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
								math.degrees(yaw_camera))
			cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			y_distance = abs(y_position) + 20 
			alt = vehicle.location.global_relative_frame.alt * 100
			# trajectory_angle = abs(math.degrees(math.atan(alt/y_distance))) #by real distance
			trajectory_angle = abs(math.degrees(math.atan(pos_camera[2]/pos_camera[1]))) #by camera distance
			cv2.putText(frame, "tarjectory angle: %4.0f"%(trajectory_angle), (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			adjust_elevator(trajectory_angle)

		# if is_detected == True:
		# 	now = datetime.now()
		# 	if last_checked <= (now - INTERVAL):
		# 		last_checked = now
		# 		# print("last_checked: ", last_checked)
		# 		adjust_elevator(trajectory_angle, is_deepstalled)
		# 		is_detected = False


		# --- write video
		out.write(frame)

		# --- Display the frame
		cv2.imshow('frame', frame)

		#-- flare
		key = cv2.waitKey(1) & 0xFF
		# current_altitude = vehicle.location.global_relative_frame.alt
		printfr("ch7: " + str(vehicle.channels['7']))
		if (int(vehicle.channels['7']) > 1514) or (key == ord('q')):
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			printfr("Saved video")
			break	

def printfr(str):
	print(str)
	logging.info(str)

def adjust_elevator(trajectory_angle):
	global angle_to_be_adjusted
	printfr("Check adjust elevator condition")
	printfr("is_deepstalled(adjust_elevator): "+ str(is_deepstalled))
	printfr("ch8 adjust elevator: " + str(vehicle.channels['8']))
	if int(vehicle.channels['8']) > 1514 and is_deepstalled:
		printfr("Tarjectory angle: " + str(trajectory_angle))
		if trajectory_angle >= 1 and trajectory_angle <= 9:
			vehicle.channels.overrides['2'] = 1515
			angle_to_be_adjusted = 1515
			printfr("adjust elevator angle to: 0 deg")
			# time.sleep(1)
		if trajectory_angle > 9 and trajectory_angle <= 18:
			vehicle.channels.overrides['2'] = 1712
			angle_to_be_adjusted = 1712
			printfr("adjust elevator angle to: -10 deg")
			# time.sleep(1)
		if trajectory_angle > 18 and trajectory_angle <= 38:
			vehicle.channels.overrides['2'] = 1880
			angle_to_be_adjusted = 1880
			printfr("adjust elevator angle to: -20 deg")
			# time.sleep(1)
		if trajectory_angle > 38:
			vehicle.channels.overrides['2'] = 1930
			angle_to_be_adjusted = 1930
			printfr("adjust MAX elevator angle")
			# time.sleep(1)

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

	video_filename = "../../../Videos/flight/" + timestr + ".avi"
	# workbook = xlsxwriter.Workbook("log/" + timestr + "_log.xlsx")

	cap = cv2.VideoCapture(0)

	if (cap.isOpened() == False):
		print("Error reading video file")
		
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))

	size = (frame_width, frame_height)
	
	out = cv2.VideoWriter(video_filename,
							cv2.VideoWriter_fourcc(*'MJPG'),
							10, size)	

	# printfr(timestr)		
	_thread.start_new_thread( image_processing, (cap, id_to_find, out, ))
	_thread.start_new_thread( deepstall, ())
	


 
except:
	print("Error: unable to start thread")

while 1:
	pass
