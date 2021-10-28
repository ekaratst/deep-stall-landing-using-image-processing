from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

connection_string = "/dev/ttyACM0"
baud_rate = 57600

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

#-- elevator-> radio2 Radio IN normal=1523 up=1924 down=1104

#--- Define Tag
id_to_find  = 72
marker_size  = 10 #- [cm]

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


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

font = cv2.FONT_HERSHEY_PLAIN

isstarted = False
const = 180 / math.pi
aoa_rad = vehicle.attitude.pitch
aoa_deg  = aoa_rad * const
total_distance = 5 

while True:
    airspeed = vehicle.airspeed
    if airspeed >= 1 and isstarted == False:
        start = time.time()
        print("start time.")
        isstarted = True

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
		
		time.sleep(5)
        if y_position <= -7:
            vehicle.channels.overrides['2'] = 1924
        elif y_position >= 7:
            vehicle.channels.overrides['2'] = 1104
        else:
            vehicle.channels.overrides['2'] = 1523
        
        airpeed_x = airspeed * (math.cos(aoa_rad))
        end = time.time()
        duration = end -start
        distance_x = airpeed_x * duration
        target_distance_x = total_distance - distance_x
        distance_z = vehicle.location.global_relative_frame
        target_distance = math.sqrt(distance_z**2 + target_distance_x**2)
       
    

	# --- Display the frame
    cv2.imshow('frame', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
