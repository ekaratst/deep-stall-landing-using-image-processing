from __future__ import print_function
from dronekit import connect, VehicleMode, mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

# def set_servo(vehicle, servo_number, pwm_value):
# 	pwm_value_int = int(pwm_value)
# 	msg = vehicle.message_factory.command_long_encode(
# 		0, 0, 
# 		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
# 		0,
# 		servo_number,
# 		pwm_value_int,
# 		0,0,0,0,0
# 		)
# 	print(msg)
# 	vehicle.send_mavlink(msg)


	

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

while True:

	print(" Mode: %s" % vehicle.mode.name)
	if vehicle.mode.name == "AUTO":
		print("AUTO!!!")
	# msg =  vehicle.message_factory.mav_cmd_do_set_servo_encode(7,1900)
	# vehicle.send_mavlink(msg)

	#print (" Ch7: %s" % vehicle.channels['7'])
	#print(vehicle.message_factory.command_long_encode())
	# set_servo(vehicle, 7, 1500)
	time.sleep(1)

