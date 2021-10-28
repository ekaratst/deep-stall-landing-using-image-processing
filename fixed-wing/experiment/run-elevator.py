#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse

connection_string = "/dev/ttyACM0"
baud_rate = 57600

#-------------(deg) 3      0     -5     -10    -15   -20
simulate_angle = [52.5, 49.82, 45.51, 40.33, 35.11, 28.86]
ch2in = [1472, 1525, 1639, 1741, 1870, 2028]
delta_angle = [3, 0, -5, -10, -15, -20]

"""
#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
simulate_angle = [52.5, 49.82, 45.51, 40.33, 35.11, 28.86, 23.81]
radio_in_elevator = [1467, 1478, 1602, 1683, 1764, 1838, 1924] #ch2in radio calibration
delta_angle = [3, 0, -5, -10, -15, -20, -25]
"""
timer_exit = 10

#--- Define Tag
id_to_find  = 72
marker_size  = 49.8 #- [cm]

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


def deepstall(cap, out, isnot_deepstalled):
	print("Thread-2")
	start = time.time()
	while True:
		print("ch7: ", vehicle.channels['7'])
		# if int(vehicle.channels['7']) > 1514 and isnot_deepstalled:
			# isnot_deepstalled = False
			# print("toogled deepstall")

			# #-- pitch down
			# pitch_down_start_time = time.time()
			# while True:
			# 	current_alttitude = vehicle.location.global_relative_frame.alt
			# 	pitch_angle = math.degrees(vehicle.attitude.pitch) 
			# 	pitch_down_now_time = time.time()
			# 	print("pitch angle: ", pitch_angle)
			# 	if pitch_down_now_time - pitch_down_start_time >= 5:
			# 		print("time out pitch down")
			# 		break
			# 	if current_alttitude <= 10.5 and current_alttitude >= 3:
			# 		print("Reached deepstall altitude.")
			# 		break
			# 	if pitch_angle  >= -12:
			# 		print("Elevator down")
			# 		vehicle.channels.overrides['2'] = 1384
			# 	else:
			# 		vehicle.channels.overrides['2'] = 1521
			# 		print("Pitch angle has been adjusted to -15[deg]")
			# 		continue

			# #-- deepstall 
			# pitch_up_start_time = time.time()
			# while True:
			# 	pitch_angle = math.degrees(vehicle.attitude.pitch) 
			# 	pitch_up_now_time = time.time()
			# 	print("pitch angle: ", pitch_angle)
			# 	if pitch_up_now_time - pitch_up_start_time >= 5:
			# 		print("time out pitch up")
			# 		break
			# 	if pitch_angle  <= 27:
			# 		print("Elevator up")
			# 		vehicle.channels.overrides['2'] = 1924
			# 	else:
			# 		vehicle.channels.overrides['2'] = 1521
			# 		print("Pitch angle has been adjusted to 60[deg]")
			# 		break
		
		# if int(vehicle.channels['7']) < 1514:
		# 	isnot_deepstalled = True

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
			
			str_position = "MARKER Position x=%4.0f z=%4.0f z=%4.0f"%(x_position, y_position, z_position)
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

			trajectory_angle = abs(math.degrees(math.atan(pos_camera[2]/pos_camera[1])))
			cv2.putText(frame, "tarjectory angle: %4.0f"%(trajectory_angle), (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			adjustElevator(trajectory_angle)
			
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
		now = time.time()
		time_ago = now -start
		if (vehicle.location.global_relative_frame.alt <= 1 and time_ago > 60*3) or (key == ord('q')):
			vehicle.channels.overrides['2'] = 1924
			# time.sleep(3)
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			break	

def adjustElevator(trajectory_angle):
	if int(vehicle.channels['7']) > 1514:
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



try:
	parser = argparse.ArgumentParser()
	parser.add_argument("number_of_run")
	args = parser.parse_args()

	video_filename = "../../../Videos/ground/28-10-64_detect_test_" + args.number_of_run + ".avi"
	# video_filename = "../../../Videos/flight/29-10-64_deepstall_ele_test_" + args.number_of_run + ".avi"
	
	cap = cv2.VideoCapture(0)

	if (cap.isOpened() == False):
		print("Error reading video file")
		
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))

	size = (frame_width, frame_height)
	
	out = cv2.VideoWriter(video_filename,
							cv2.VideoWriter_fourcc(*'MJPG'),
							10, size)
						
	# _thread.start_new_thread( timer, (cap, out, ))
	_thread.start_new_thread( deepstall, (cap, out, isnot_deepstalled, ))
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
