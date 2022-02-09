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

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

isnot_deepstalled = True
is_deepstalled = False

def deepstall(is_deepstalled,row, col, ratio_time, cap, out):
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
		print("ch8: ", vehicle.channels['8']) # G switch
		print("distance: ", get_distance_metres(current_waypoint_location, target_waypoint_location))
		if int(vehicle.channels['8']) > 1514 and not is_deepstalled: # toggle when enter auto mode
			if get_distance_metres(current_waypoint_location, target_waypoint_location) <= 25: #9
				poststall_waypoint_location = vehicle.location.global_relative_frame
				print(is_deepstalled)
				vehicle.mode = VehicleMode("STABILIZE")
				vehicle.channels.overrides['2'] = 1800
				# if n_deepstall == 0:
				start_time = time.time()
				is_deepstalled = True
				print("Deep stall!!!")	

		if int(vehicle.channels['8']) < 1514:
			is_deepstalled = False
			vehicle.channels.overrides = {}
			print("Pilot control")
		
		if int(vehicle.channels['8']) > 1514 and is_deepstalled:
			# post_stall(start_time, row ,col, ratio_time)
			vehicle.channels.overrides['2'] = 1800
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
			if current_altitude <= 1 and int(vehicle.channels['8']) > 1514:
				workbook.close()
				print("end log!!")
				
		print("-------------------------------------")
		# time.sleep(1)
		#-- detect and adjust
			
		#-- Read the camera frame
		ret, frame = cap.read()

		#-- Convert in gray scale
		gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

		#-- Find all the aruco markers in the image
		corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
								cameraMatrix=camera_matrix, distCoeff=camera_distortion)
		
		if ids is not None and ids[0] == id_to_find:
			
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
			roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

			#-- Print the marker's attitude respect to camera frame
			str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
								math.degrees(yaw_marker))
			cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


			#-- Now get Position and attitude f the camera respect to the marker
			pos_camera = -R_tc*np.matrix(tvec).T

			str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
			cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#-- Get the attitude of the camera respect to the frame
			roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
			str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
								math.degrees(yaw_camera))
			cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			y_distance = abs(y_position) + 20 
			alt = vehicle.location.global_relative_frame.alt * 100
			# trajectory_angle = abs(math.degrees(math.atan(alt/y_distance))) #by real distance
			trajectory_angle = abs(math.degrees(math.atan(pos_camera[2]/pos_camera[1]))) #by camera distance
			cv2.putText(frame, "tarjectory angle: %4.0f"%(trajectory_angle), (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
		
		# else:
		# 	if is_deepstalled:
		# 		vehicle.channels.overrides['2'] = 2114
		# 		print("Elevator up2")

			# adjustElevator(trajectory_angle, is_deepstalled)
			
		# else:
		# 	vehicle.channels.overrides['2'] = 1500

		# --- write video
		out.write(frame)

		# --- Display the frame
		cv2.imshow('frame', frame)

		# key = cv2.waitKey(1) & 0xFF
		# if key == ord('q'):
		# 	cap.release()
		# 	out.release()
		# 	cv2.destroyAllWindows()
		# 	break	

		#-- flare
		key = cv2.waitKey(1) & 0xFF
	
		print("ch7: ", vehicle.channels['7'])
		if (int(vehicle.channels['7']) > 1514) or (key == ord('q')):
			# vehicle.channels.overrides['2'] = 1924
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			break	

def adjustElevator(trajectory_angle, is_deepstalled):
	if vehicle.mode.name == "AUTO" and is_deepstalled:
	# if int(vehicle.channels['8']) > 1514:
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

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

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
	# video_filename = "../../../Videos/ground/20-12-64_ground_test_" + args.number_of_run + ".avi"
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
	_thread.start_new_thread( deepstall, (is_deepstalled, row, col, ratio_time, cap, out,))
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
