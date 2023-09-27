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

    return angle_deg

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

def Marker_length(h):
    arucoMarkerLength = 629.81 * (145/h)

    return arucoMarkerLength

def intrinsic():
    f = 629.81 # from focal calculations
    width = 800 
    height = 600

    intrinsic_matrix = np.matrix([
                        [f,0,width/2],
                        [0,f,height/2],
                        [0,0,1]
                        ])

    return intrinsic_matrix    


def GetDegreesFromVector(tvecs):
    beta = np.arccos(np.dot((tvecs/np.linalg.norm(tvecs)),[0,0,1]))
    angleRad = beta * np.sign(np.dot(tvecs, [1,0,0]))
    angleDeg = 180 * angleRad / math.pi
    return angleDeg


def Beta(tvecs):
    beta = np.arccos(np.dot((tvecs/np.linalg.norm(tvecs)),[0,0,1]))
    PosNeg = beta * np.sign(np.dot(tvecs, [1,0,0]))
    return PosNeg

def rad2degrees(beta):
    angle = beta*np.array([1,0,0])
    angle = np.degrees(angle)
    print(f"angle degrees: {angle}")
    
    return angle
 

def calc_h(aruco_corners):
    h = aruco_corners[0][0][3][1] - aruco_corners[0][0][1][1]
    return h

# def calc_Z(h,f):
#     Z = f* (145/h)
#     return Z


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
FPS = 15
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

pprint(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(2)  # wait for camera to setup

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

def DetectTarget():
    retvaldir = 1000 
    retvaldist = 1000
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        for _ in range(5):

            try:
                print("start")
                image = cam.capture_array("main")
            
                cv2.imshow(WIN_RF, image)

                aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)
                h = calc_h(aruco_corners)
                arucoMarkerLength = Marker_length(h)
                intrinsic_matrix = intrinsic()
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
                print(f"id: {ids} \n tvec:{tvecs}")

                # dir = Beta(tvecs[0][0])
                # dir = rad2degrees(dir)
                # dir = dir[0]

                dir = angle_between_vectors(np.array([tvecs[0][0][0],tvecs[0][0][2]]),np.array([0,1]))

                dist = np.linalg.norm(tvecs)
                enddist = predict_t_values((dist/100))

                retvaldir = dir
                retvaldist = enddist
            except:
                if (retvaldir != 1000):
                    retvaldir = 1000 
                    retvaldist = 1000
                else: 
                    pass
            
        return retvaldir, retvaldist


def TurnNGo():
    currDir, currDist = DetectTarget()
    print("arlo should drive",currDist/100)
    print("arlo should angle",currDir)
    arlo.RotateAngle(currDir)
    arlo.DriveLength(currDist/100)

# TurnNGo()


def SearchNTurnNGo():
    while True:
        currDir, currDist = DetectTarget()

        if (currDir != 1000):
            print("arlo should drive",currDist/100)
            print("arlo should angle",currDir)
            arlo.RotateAngle(currDir)
            arlo.DriveLength(currDist/100)
            break
        else:
            print("here")
            arlo.RotateAngle(20)





def DetectTargetContinous():
    while cv2.waitKey(4) == -1: # Wait for a key pressed event

        try:
            print("start")
            image = cam.capture_array("main")
        
            cv2.imshow(WIN_RF, image)

            aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)
         
            h = calc_h(aruco_corners)
            arucoMarkerLength = Marker_length(h)
            intrinsic_matrix = intrinsic()
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
            
            print(f"id: {ids} \n tvec:{tvecs}")

            # dir = Beta(tvecs[0][0])
            # dir = rad2degrees(dir)
            # dir = dir[0]

            # andersnewdir = GetDegreesFromVector(tvecs[0][0])
            # print(f"andersnewdir={andersnewdir}")
            andersolddir = angle_between_vectors(np.array([tvecs[0][0][0],tvecs[0][0][2]]),np.array([0,1]))
            print(f"andersolddir={andersolddir}")
            # dir = Beta(tvecs[0][0])
            # dir = rad2degrees(dir)
            # mmdir = dir[0]
            # print(f"mmdir={mmdir}")

            # dist = np.linalg.norm(tvecs)
            # enddist = predict_t_values((dist/100))
            # print(f"enddist={enddist}")

        except:
            pass


# DetectTargetContinous()
def MapTargets():

    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        
        KnownTvecs = []
        KnownIDs = []

        for _ in range(5):

            try:
                print("start")
                image = cam.capture_array("main")
            
                cv2.imshow(WIN_RF, image)

                aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)

                if (len(KnownIDs) == 0): #if ids longer than knowids, append and remove dupes
                    KnownIDs.extend(ids)

                h = calc_h(aruco_corners)
                arucoMarkerLength = Marker_length(h)
                intrinsic_matrix = intrinsic()
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
                
                print("tvecs in inner loop:", tvecs)
                if (len(KnownTvecs) == 0):
                    KnownTvecs.extend(tvecs)
                
            except:
                    pass
            
        retVal = []
        print(f"tvecs inhold: {KnownTvecs}\n length KnownTvecs {len(KnownTvecs)}")
        print(f"Ids inhold: {KnownIDs}\n length KnownIds {len(KnownIDs)}")

        for i in range(len(KnownIDs)):

            print("ids",ids)
            currID = KnownIDs[i][0]
            currTvec = KnownTvecs[i][0]
            print("currTvecs",currTvec)

            currTvec2D = np.array([currTvec[0],currTvec[2]]) #remove y
            FixedCurrTvec = (currTvec2D / np.linalg.norm(currTvec2D))*np.linalg.norm(currTvec)

            retVal.append((FixedCurrTvec,currID))
            print("retval",retVal)

        return retVal

# SearchNTurnNGo()

MapTargets()
print(MapTargets())


