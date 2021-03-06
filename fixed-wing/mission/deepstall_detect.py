from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, threading

connection_string = "/dev/ttyACM0"
baud_rate = 57600

#1678 -> 10 deg
#1507 -> Neutral
#1186 -> -30 deg
#-------------(deg) 3      0     -5     -10    -15   -20    -25    -30  
simulate_angle = [50.84, 48.48, 44.57, 40.43, 36.66, 32.11, 28.41, 26.18]
radio_in_elevator = [1558, 1507, 1451, 1398, 1345, 1292, 1239, 1186]
delta_angle = [3, 0, -5, -10, -15, -20, -25, -30]
timer_exit = 10

print('Connecting to Vehicle on: %s' %connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
vehicle.wait_ready('autopilot_version')

#-- elevator-> radio2 Radio IN normal=1523 up=1924 down=1104

#--- Define Tag
id_to_find  = 72
marker_size  = 10 #- [cm]

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
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #1280
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) #720

out = cv2.VideoWriter('video_test.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, (640,480))

font = cv2.FONT_HERSHEY_PLAIN

experimental_height = 12 #12
flare_altitude = 1.2 #1.2
quit_program_altitude = 0.5 #0.5 
current_altitude = vehicle.location.global_relative_frame.alt
check_flare_altitude = current_altitude - experimental_height + flare_altitude
check_quit_program_altitude = current_altitude - experimental_height + quit_program_altitude

t = threading.Timer(timer_exit, exit_program)
t.start()

print("++Start++")
time.sleep(4)
print("Deep stall")
vehicle.channels.overrides['2'] = radio_in_elevator[7] #deepstall
time.sleep(1)


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

        #if y_position <= -7:
        #    vehicle.channels.overrides['2'] = 1924 #elevator-up
        #elif y_position >= 7:
        #    vehicle.channels.overrides['2'] = 1104 #elevator-down
        #else:
        #    vehicle.channels.overrides['2'] = 1523
        #time.sleep(5)


        for i in range(8):
            if trajectory_angle >= simulate_angle[i] or trajectory_angle <= simulate_angle[7]:
                vehicle.channels.overrides['2'] = radio_in_elevator[i]
                state_elevator = radio_in_elevator[i]
                print(radio_in_elevator[i])
                print("adjust elevator angle to: " , delta_angle[i], " deg")
                break
    else:
        vehicle.channels.overrides['2'] = radio_in_elevator[7]
    # vehicle.channels.overrides['2'] = 2088
    # time.sleep(5)
    # vehicle.channels.overrides['2'] = 940
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    
    if vehicle.location.global_relative_frame.alt <= check_flare_altitude:
         vehicle.channels.overrides['2'] = radio_in_elevator[7]

    # --- write video
    out.write(frame)

	# --- Display the frame
    cv2.imshow('frame', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    # if vehicle.location.global_relative_frame.alt <= check_quit_program_altitude:
    if key == ord('q'):
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        break
   


