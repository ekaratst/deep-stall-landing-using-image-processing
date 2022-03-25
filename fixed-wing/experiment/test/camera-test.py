#!/usr/bin/python3

from __future__ import print_function
from dronekit import connect, VehicleMode
import numpy as np
import cv2

cap = cv2.VideoCapture(0)


if (cap.isOpened() == False):
    print("Error reading video file")

while True:
    ret, frame = cap.read()	
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break	
