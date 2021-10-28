#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread, argparse

def deepstall(check_deepstall):
	time.sleep(5)
	print("Ready")
	if check_deepstall:
		print("Thread-2")
		time.sleep(0.5)
		print("3")
		time.sleep(1)
		print("2")
		time.sleep(1)
		print("1")
		time.sleep(2.5)
		print("Deep stall")
		check_deepstall = False

def timer(cap, out, flight_time):
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
		if now - start >= flight_time:
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			break	

try:
	parser = argparse.ArgumentParser()
	parser.add_argument("number_of_run")
	args = parser.parse_args()
	#video_filename = "videos/sikan_camera_test_" + args.number_of_run + ".avi"
	# video_filename = "~/Videos/ground/7-10-64_camera_test_" + args.number_of_run + ".avi"
	video_filename = "../../../Videos/ground/11-10-64_camera_test_" + args.number_of_run + ".avi"	
	check_deepstall = True
	flight_time = 420
	cap = cv2.VideoCapture(0)

	if (cap.isOpened() == False):
		print("Error reading video file")
		
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))

	size = (frame_width, frame_height)
	
	out = cv2.VideoWriter(video_filename,
							cv2.VideoWriter_fourcc(*'MJPG'),
							12, size)
						
	_thread.start_new_thread( timer, (cap, out, flight_time))
	#_thread.start_new_thread( deepstall, (check_deepstall, ))
	
 
except:
	print ("Error: unable to start thread")

while 1:
	pass
