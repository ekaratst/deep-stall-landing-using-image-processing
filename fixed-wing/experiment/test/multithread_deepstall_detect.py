#!/usr/bin/python3


from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, _thread

def save_video(size):
	out = cv2.VideoWriter('filename3.avi',
							cv2.VideoWriter_fourcc(*'MJPG'),
							10, size)
	# --- write video
    print("start saving")
    out.write(frame)

def detect_aruco(simulate_angle, radio_in_elevator, delta_angle, vehicle, id_to_find, marker_size, calib_path, camera_matrix, camera_distortion, R_flip, aruco_dict, parameters, cap, size, font):
	while True:

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

    #     for i in range(8):
    #         if trajectory_angle >= simulate_angle[i] or trajectory_angle <= simulate_angle[7]:
    #             vehicle.channels.overrides['2'] = radio_in_elevator[i]
    #             state_elevator = radio_in_elevator[i]
    #             print(radio_in_elevator[i])
    #             print("adjust elevator angle to: " , delta_angle[i], " deg")
    #             break
    # else:
    #     vehicle.channels.overrides['2'] = radio_in_elevator[7]

    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    # print("Flare: ", check_flare_altitude)
    
    # if vehicle.location.global_relative_frame.alt <= check_flare_altitude:
    #      vehicle.channels.overrides['2'] = radio_in_elevator[7]


	# --- Display the frame
    cv2.imshow('frame', frame)
    print("---",round,"---")

    # print("first round")
    if round == 20:
    
        # t = threading.Timer(timer_exit, exit_program)
        # t.start()
        print("3")
        time.sleep(1)
        print("2")
        time.sleep(1)
        print("1")
        time.sleep(2.5)
        #print("++Start++")
        #time.sleep(2)
        print("Deep stall")
        vehicle.channels.overrides['2'] = radio_in_elevator[7] #deepstall
        time.sleep(1)
        check_first_round = False
	
		#deep_stall(timer_exit)
    round = round + 1
    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    # if vehicle.location.global_relative_frame.alt <= check_quit_program_altitude:
    if key == ord('q'):
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        break	

	
def exit_program():
    print("cap release")
    cap.release()
    print("out release")
    out.release()
    print("destroyAllWindows")
    cv2.destroyAllWindows()

# Checks if a matrix is a valid rotation matrix.
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

# Define a function for the thread
def print_time( threadName, delay):
   count = 0
   while count < 5:
      time.sleep(delay)
      count += 1
      print ("%s: %s" % ( threadName, time.ctime(time.time()) ))

# Create two threads as follows
try:

   _thread.start_new_thread( print_time, ("Thread-1", 2, ) )
   _thread.start_new_thread( print_time, ("Thread-2", 4, ) )
   
	connection_string = "/dev/ttyACM0"
	baud_rate = 57600

	#1678 -> 10 deg
	#1507 -> Neutral
	#1186 -> -30 deg
	#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
	simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41, 26.18]
	radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
	delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]
	timer_exit = 20

	print('Connecting to Vehicle on: %s' %connection_string)
	vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
	vehicle.wait_ready('autopilot_version')

	#--- Define Tag
	id_to_find  = 72
	marker_size  = 50 #- [cm]   

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
	
	
	cap = cv2.VideoCapture(0)

	if (cap.isOpened() == False):
		print("Error reading video file")
		
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))

	size = (frame_width, frame_height)
	
	#-- Set the camera size as the one it was calibrated with
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width) #1280, 640
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height) #720, 480

	font = cv2.FONT_HERSHEY_PLAIN
	
	
except:
   print ("Error: unable to start thread")

while 1:
   pass
