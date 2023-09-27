# This script shows how to open a camera the picamera2 module and grab frames and show these.
# Kim S. Pedersen, 2023

import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *
import SmartArloNew as arlo

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)


arlo = arlo.betterRobot()


def angle_between_vectors(vector1, vector2):
    # Calculate the dot product of the two vectors
    dot_product = np.dot(vector1, vector2)

    # Calculate the magnitude (norm) of each vector
    magnitude_vector1 = np.linalg.norm(vector1)
    magnitude_vector2 = np.linalg.norm(vector2)

    # Calculate the cosine of the angle between the vectors using the dot product formula
    cosine_theta = dot_product / (magnitude_vector1 * magnitude_vector2)

    # Calculate the angle in radians using the arccosine function
    angle_rad = np.arccos(cosine_theta)

    # Determine the sign of the angle (clockwise or counterclockwise)
    cross_product = np.cross(vector1, vector2)
    if cross_product < 0:
        angle_rad = -angle_rad

    # Convert the angle to degrees
    angle_deg = np.degrees(angle_rad)

    return -angle_deg

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
    return F

f = 629.81
def Marker_length(h):
    real_Height = 300 #obstacle height in mm
    # smallX = [380,308,266,234,206,187,173,159,144,134,128,120,113,107] # størrelse af objekt på billede i pixels

    # for i in range(len(smallX)):
    # arucoMarkerLength = 629.81 * (300/smallX[i])
    arucoMarkerLength = 629.81 * (145/h)
        # print("arucoMarkerLength: ",arucoMarkerLength)
    return arucoMarkerLength

def intrinsic():
    f = 629.81 # from focal calculations
    width = 800 
    height = 600
    # hardcodede for test
    
    x,y,z = 1,0,0
    # intrinsic_matrix =np.dot(np.matrix([
    #                     [f,0,0,width/2],
    #                     [0,f,0,height/2],
    #                     [0,0,1,0]
    #                     ]),[x,y,z,1])

    # intrinsic_matrix = np.matrix([
    #                     [f,0,0,width/2],
    #                     [0,f,0,height/2],
    #                     [0,0,1,0]
    #                     ])

    intrinsic_matrix = np.matrix([
                        [f,0,width/2],
                        [0,f,height/2],
                        [0,0,1]
                        ])

    # print (f"int matrix; {intrinsic_matrix}")
    return intrinsic_matrix    


def Beta(tvecs):
    beta = np.arccos(np.dot((tvecs/np.linalg.norm(tvecs)),[0,0,1]))
    PosNeg = beta * np.sign(np.dot(tvecs, [1,0,0]))
    return PosNeg

def turn_angle(beta):
    angle = beta*np.array([1,0,0])
    angle = np.degrees(angle)
    print(f"angle degrees: {angle}")
    
    return angle
 

def calc_h(aruco_corners):
    # print("aruco_corners", aruco_corners)
    # print(f"arc[0]", aruco_corners[0])
    # print(f"arc[0][0]", aruco_corners[0][0])
    # print(f"arc[0][0][0]", aruco_corners[0][0][0])
    # print(f"arc[0][0][0][1]", aruco_corners[0][0][0][1])
    h = aruco_corners[0][0][3][1] - aruco_corners[0][0][1][1]
    return h

def calc_Z(h,f):
    Z = f* (145/h)
    return Z


def predict_t_values(X):
    X = X/100
    # Get the coefficients and y-intercept from the polynomial model
    coef = [0.00000000e+00, -2.22966720e-03, 7.68889479e-05]
    intercept = 0.0993244545876042

    # Calculate the predicted t-values using the inverse transformation of the polynomial features
    t_predicted = (-coef[1] + np.sqrt(coef[1]**2 - 4*coef[2]*(intercept - X))) / (2*coef[2])

    return t_predicted







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



# focal() # has been calculated already and data extracted

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
dist = 0
dir = 0

# for i in [1]:
def DetectTarget():
# while cv2.waitKey(4) == -1: # Wait for a key pressed event
#if (True):
    for i in range(5):

        try:
            print("start")
            image = cam.capture_array("main")
            
            # Show frames
            cv2.imshow(WIN_RF, image)

            # print(f"ArucoMarkerLength: {arucoMarkerLength}\n")
            aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)
            h = calc_h(aruco_corners)
            arucoMarkerLength = Marker_length(h)
            # Z = calc_Z(calc_h(aruco_corners),f)
            # print("Z", Z)

            # print(f"aruco_corners =\n {aruco_corners} \n\n ids =\n {ids} \n\n rejectImgPoints =\n {rejectedImgPoints}\n")
            # distortion_coeffs = None # we dont know the distortion 
            intrinsic_matrix = intrinsic()
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
            # print(f"tvecs: {tvecs} \n tvecs.shape: {tvecs.shape}\n tvecs[0]: {tvecs[0]}\n")
            # print("rvecs: \n",rvecs)
            # print("objPoints:\n ",objPoints)

            # angle_between_vectors(tvecs[0][0],[0,0,1])
            # print(f"Angle is :\n{angle_between_vectors(tvecs,[0,0,1])}")
            Beta(tvecs)
            # dir = turn_angle(Beta(tvecs))
            # print("aruco corners", aruco_corners)
            dir = Beta(tvecs[0][0])
            dir = turn_angle(dir)
            print("turn dir",dir)
            dir = dir[0]
            
            


        
            # print("dir = tvecs[0][0] ", dir)
            # dir = np.array([dir[0], dir[2]])
            dist = np.linalg.norm(tvecs)
            print(f"\n\nDist: {dist/100}\n\n")
            enddist = predict_t_values((dist/100))
            print(f"calculated dist in cm: {enddist}")
            dist = enddist
            # print(f"\n\nDist: {np.linalg.norm(tvecs[0][2])}\n\n")
            # print(f"\n\nDist: {np.linalg.norm(tvecs)[0][0]}\n\n")
            # for i in tvecs:
            #     print(f"\n\n i: {i} | tvecs: {np.linalg.norm(i)} \n\n")
            
            
            # print("Beta: ", Beta(tvecs))
            # print("angle",turn_angle(Beta(tvecs)))
            retvaldir = dir
            retvaldist = dist
        except:
            retvaldir = 1000 
            retvaldist = 1000
        
        return retvaldir, retvaldist



def TurnNGo():
    currDir, currDist = DetectTarget()
    print("arlo should drive",currDist/100)
    print("arlo should angle",currDir)
    arlo.RotateAngle(currDir)
    arlo.DriveLength(currDist/100)

TurnNGo()

def SearchNTurnNGo():
    while True:
        currDir, currDist = DetectTarget()

        if (currDir != 1000):
            print("arlo should drive",currDist/100)
            print("arlo should angle",currDir)
            arlo.RotateAngle(-currDir)
            arlo.DriveLength(currDist/100)
            break
        else:
            print("here")
            arlo.RotateAngle(20)









