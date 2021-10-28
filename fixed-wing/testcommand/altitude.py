from dronekit import connect, VehicleMode
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

while 10:
    # Print location information for `vehicle` in all frames (default printer)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)    #NED
    print("\n")
    # Print altitudes in the different frames (see class definitions for other available information)
    print("Altitude (global frame): %s" % vehicle.location.global_frame.alt)
    print("Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt)
    print("Altitude (NED frame): %s" % vehicle.location.local_frame.down)
    print("\n")
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Sea level altitude is: %s" % vehicle.location.global_frame.alt)
    print("--------------------------------------\n")
    time.sleep(1)