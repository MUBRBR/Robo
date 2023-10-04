import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *
import SmartArloNew as arlo
from Task3_3virker import *
from grid_occ import *



def CreateLandmarksMap():

    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        
        KnownTvecs = []
        KnownIDs = []

        for _ in range(5):

            try:
                image = cam.capture_array("main")
            
                cv2.imshow(WIN_RF, image)

                aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)

                h = calc_h(aruco_corners)
                arucoMarkerLength = Marker_length(h)
                intrinsic_matrix = intrinsic()
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
                
                if (len(KnownTvecs) == 0):
                    KnownTvecs.extend(tvecs)
                
            except:
                    pass
            
        retVal = []

        for i in range(len(KnownIDs)):
            currTvec = KnownTvecs[i][0]
            currTvec2D = np.array([currTvec[0],currTvec[2]]) #remove y
            FixedCurrTvec = (currTvec2D / np.linalg.norm(currTvec2D))*(np.linalg.norm(currTvec)/100)

            retVal.append((FixedCurrTvec))

        return retVal

# landmarksMap = CreateLandmarksMap()
# landmarksMap = np.array([[0,200],[-100,200],[100,200]])

def EvaluateCollisionLandmarksMap(landmarksMap, pos):

    print(f"landmarksmap: {landmarksMap}")
    for elm,_ in landmarksMap:
        distance = np.sqrt((pos[0]-elm[0])**2+(pos[1]-elm[1])**2)
        print(f"Distance: {distance}")
        required_distance = 34 + 20 #obstacle radius + arlo-radius

        if (distance <= required_distance):
            return True

    return False

testPos = [1,0]

map = GridOccupancyMap(low=(0, 0), high=(3, 4), res=path_res)
map.register_obstacle(CreateLandmarksMap())
map.in_collision([0,0])
map.draw_map()




