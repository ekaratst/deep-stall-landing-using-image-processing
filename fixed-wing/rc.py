#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode
import time

connection_string = "/dev/ttyACM0"
baud_rate = 57600

# Connect to the Vehicle. 
print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

vehicle.wait_ready('autopilot_version')

#elevator->channel2 up=1635 down=1320
#--------elevator->radio2 Radio IN normal=1523 up=1924 down=1104---------
#elevator->radio2 SerovoMotor Out normal=1490 up=1636 down=1320

# Get all channel values from RC transmitter
print("Channel values from RC Tx:", vehicle.channels)

# Access channels individually
print ("Read channels individually:")
print (" Ch1: %s" % vehicle.channels['1'])
print (" Ch2: %s" % vehicle.channels['2'])

vehicle.channels.overrides['2'] = 1924
time.sleep(1)
vehicle.channels.overrides['2'] = 1523
time.sleep(1)
vehicle.channels.overrides['2'] = 1104
time.sleep(1)
vehicle.channels.overrides['2'] = 1523

print (" Ch1: %s" % vehicle.channels['1'])
print (" Ch2: %s" % vehicle.channels['2'])
