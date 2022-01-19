#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse

connection_string = "/dev/ttyACM0"
baud_rate = 57600

#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
# simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41, 26.18]
# radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
# delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]
#[1924,1500,900]

# simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41]
simulate_angle = [52.5, 49.82, 45.51, 40.33, 35.11, 28.86, 23.81]
radio_in_elevator = [1467, 1478, 1602, 1683, 1764, 1838, 1924] #ch2in radio calibration
delta_angle = [3, 0, -5, -10, -15, -20, -25]
timer_exit = 10

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

def deepstall(check_deepstall):
	print("Thread-2")
	time.sleep(1)
	#if check_deepstall:
	while True:
		# print("Checking Guided Mode...")
		# print(" Mode: %s" % vehicle.mode.name)
		# if vehicle.mode.name=='STABILIZE':
		print("ch7: ", vehicle.channels['7'])
		
		if int(vehicle.channels['7']) > 1514:
			print("Deep stall mode")
			# print("starting deepstall")
			# time.sleep(0.5)
			# print("3")
			# time.sleep(1)
			# print("2")
			# time.sleep(1)
			# print("1")
			# time.sleep(2.5)
			pitch_down_start_time = time.time()
			while True:
				pitch_angle = math.degrees(vehicle.attitude.pitch) 
				pitch_down_now_time = time.time()
				print("pitch angle: ", pitch_angle)
				if pitch_down_now_time - pitch_down_start_time >= 5:
					print("time out pitch down")
					break
				if pitch_angle  >= -12:
					print("Elevator down")
					vehicle.channels.overrides['2'] = 1312
				else:
					vehicle.channels.overrides['2'] = 1500
					print("Pitch angle has been adjusted to -15[deg]")
					break
				time.sleep(0.5)
			time.sleep(3)
			pitch_up_start_time = time.time()
			while True:
				pitch_angle = math.degrees(vehicle.attitude.pitch) 
				pitch_up_now_time = time.time()
				print("pitch angle: ", pitch_angle)
				if pitch_up_now_time - pitch_up_start_time >= 5:
					print("time out pitch up")
					break
				if pitch_angle  <= 27:
					print("Elevator up")
					vehicle.channels.overrides['2'] = 1924
				else:
					vehicle.channels.overrides['2'] = 1500
					print("Pitch angle has been adjusted to 60[deg]")
					break

			# time.sleep(0.5)
			# print("Deep stall")
			# vehicle.channels.overrides['2'] = radio_in_elevator[6] #deepstall
			# time.sleep(1)
			break
			#check_deepstall = False

def timer(cap, out):
	print("Thread-1")
	print("start time")
	start = time.time()
	while True:
		ret, frame = cap.read()	
		out.write(frame)
		cv2.imshow('frame', frame)
		now = time.time()
		key = cv2.waitKey(1) & 0xFF
		# if key == ord('q'):
		time_ago = now - start
		current_alttitude = vehicle.location.global_relative_frame.alt
		if (time_ago >= 60*10) or (time_ago >= 60 and current_alttitude <= 1):
			time.sleep(3)
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			break	

try:
	parser = argparse.ArgumentParser()
	parser.add_argument("number_of_run")
	args = parser.parse_args()

	video_filename = "../../../Videos/ground/12-10-64_sikan_test_" + args.number_of_run + ".avi"
	check_deepstall = True
	cap = cv2.VideoCapture(0)

	if (cap.isOpened() == False):
		print("Error reading video file")
		
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))

	size = (frame_width, frame_height)
	
	out = cv2.VideoWriter(video_filename,
							cv2.VideoWriter_fourcc(*'MJPG'),
							10, size)
						
	_thread.start_new_thread( timer, (cap, out, ))
	_thread.start_new_thread( deepstall, (check_deepstall, ))
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
