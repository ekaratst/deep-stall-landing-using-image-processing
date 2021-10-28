from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 10      0     -5     -10    -15   -20    -25    -30  
simulate_angle = [47.84, 45.84, 41.5, 36.96, 32.67, 27.57, 24.36, 22.31]
radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]


print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

#-- elevator-> radio2 Radio IN normal=1523 up=1924 down=1104

experimental_height = 12
flare_altitude = 1.2
quit_program_altitude = 0.5 
current_altitude = vehicle.location.global_relative_frame.alt
check_flare_altitude = current_altitude - experimental_height + flare_altitude
check_quit_program_altitude = current_altitude - experimental_height + quit_program_altitude

print("++Start++")
time.sleep(4)
print("Deep stall")
vehicle.channels.overrides['2'] = radio_in_elevator[7] #deepstall

cv2.destroyAllWindows()



