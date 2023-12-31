import numpy as np
import cv2
import math

# Create an ArUco dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)





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

focal()

def Marker_length():
    real_Height = 300 #obstacle height in mm
    smallX = [380,308,266,234,206,187,173,159,144,134,128,120,113,107] # størrelse af objekt på billede i pixels

    for i in range(len(smallX)):
        arucoMarkerLength = 625.33 * (300/smallX[i])
        print("arucoMarkerLength: ",arucoMarkerLength)
    return arucoMarkerLength

def intrinsic():
    f = 625.33 # from focal calculations
    width = 800 
    height = 600

    intrinisc_matrix =np.matrix([
                        [f,0,0,width/2],
                        [0,f,0,height/2],
                        [0,0,1,0]
                        ])
    return intrinisc_matrix    


def Beta(tvecs):
    beta = np.cos(np.dot((tvecs/math.sqrt(sum(i**2 for i in tvecs))),[0,0,1]))
    return beta


arucoMarkerLength = Marker_length()
aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, arucoDict)
distortion_coeffs = None # we dont know the distortion 
intrinsic_matrix = intrinsic()
rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, distortion_coeffs)

print(Beta(tvecs))