# This script shows how to open a camera the picamera2 module and grab frames and show these.
# Kim S. Pedersen, 2023

import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)


def focal():
    bigX = 300 # mm højde på objekt
    Z = [500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800] # distance i mm for hvert billede
    smallX = [380,308,266,234,206,187,173,159,144,134,128,120,113,107] # størrelse af objekt på billede i pixels
    F = [0]*len(Z)
    # f = x*(Z/X)
    for i in range(len(Z)):
        F[i] = smallX[i]*(Z[i]/bigX)
        print(f"F[{i}] = {F[i]}")
    print(f"F = {F}")
    average = np.mean(F)
    print(f"Average F = {average}")
    std = np.std(F)
    print(f"std of F = {std}")
    return F,smallX


def Marker_length():
    real_Height = 300 #obstacle height in mm
    smallX = [380,308,266,234,206,187,173,159,144,134,128,120,113,107] # størrelse af objekt på billede i pixels

    for i in range(len(smallX)):
        arucoMarkerLength = 625.33 * (300/smallX[i])
        # print("arucoMarkerLength: ",arucoMarkerLength)
    return arucoMarkerLength

def intrinsic():
    f = 625.33 # from focal calculations
    width = 800 
    height = 600
    # hardcodede for test
    
    x,y,z = 100,10,0
    intrinsic_matrix =np.dot(np.matrix([
                        [f,0,0,width/2],
                        [0,f,0,height/2],
                        [0,0,1,0]
                        ]),[x,y,z,1])

    # intrinsic_matrix = np.matrix([
    #                     [f,0,0,width/2],
    #                     [0,f,0,height/2],
    #                     [0,0,1,0]
    #                     ])

    print (f"int matrix; {intrinsic_matrix}")
    return intrinsic_matrix    


def Beta(tvecs):
    beta = np.cos(np.dot((tvecs/math.sqrt(sum(i**2 for i in tvecs))),[0,0,1]))
    return beta







print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
imageSize = (800, 600)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

pprint(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(1)  # wait for camera to setup


# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)



focal()
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
while cv2.waitKey(4) == -1: # Wait for a key pressed event
#if (True):
    image = cam.capture_array("main")
    
    # Show frames
    cv2.imshow(WIN_RF, image)
    arucoMarkerLength = Marker_length()
    # arucoMarkerLength = 600

    print(f"ArucoMarkerLength: {arucoMarkerLength}\n")
    aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)
    print(f"aruco_corners =\n {aruco_corners} \n\n ids =\n {ids} \n\n rejectImgPoints =\n {rejectedImgPoints}\n")
    distortion_coeffs = None # we dont know the distortion 
    intrinsic_matrix = intrinsic()
    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, [])
    print("tvecs: \n",tvecs)
    print("rvecs: \n",rvecs)
    print("objPoints:\n ",objPoints)


    print("Beta: ", Beta(tvecs))
    

# Finished successfully








