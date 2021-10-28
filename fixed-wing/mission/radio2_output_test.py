from __future__ import print_function
from dronekit import connect, VehicleMode
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

	
print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41, 26.18]
radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]
index_delta_angle = 0

print("start")
for i in radio_in_elevator:
	print("angle: ",delta_angle[index_delta_angle], " degree[",i,"]")
	#print(i)
	vehicle.channels.overrides['2'] = i
	time.sleep(1)
	index_delta_angle = index_delta_angle + 1




