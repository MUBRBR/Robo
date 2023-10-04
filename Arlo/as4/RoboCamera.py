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



class RoboCamera():
    def __init__(self):
        self.imageSize = (800, 600)
        self.WIN_RF = "Example 1"
        self.cam = picamera2.Picamera2()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.start_Camera()

    def start_Camera(self):
        print("OpenCV version = " + cv2.__version__)
        # Open a camera device for capturing
        FPS = 15
        frame_duration_limit = int(1/FPS * 1000000) # Microseconds
        # Change configuration to set resolution, framerate
        picam2_config = self.cam.create_video_configuration({"size": self.imageSize, "format": 'RGB888'},
                                                                    controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                                    queue=False)
        self.cam.configure(picam2_config) # Not really necessary
        self.cam.start(show_preview=False)

        pprint(self.cam.camera_configuration()) # Print the camera configuration in use

        time.sleep(1)  # wait for camera to setup

        # Open a window
        cv2.namedWindow(self.WIN_RF)
        cv2.moveWindow(self.WIN_RF, 100, 100)

    def angle_between_vectors(self, vector1, vector2):
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

    def Marker_length(self, h):
        arucoMarkerLength = 629.81 * (145/h)
        return arucoMarkerLength

    def intrinsic(self):
        f = 629.81 # from focal calculations
        width = 800 
        height = 600
        intrinsic_matrix = np.matrix([
                            [f,0,width/2],
                            [0,f,height/2],
                            [0,0,1]
                            ])

        return intrinsic_matrix  

    def predict_t_values(self, X):
        X = X/100
        # Get the coefficients and y-intercept from the polynomial model
        coef = [0.00000000e+00, -2.22966720e-03, 7.68889479e-05]
        intercept = 0.0993244545876042

        # Calculate the predicted t-values using the inverse transformation of the polynomial features
        t_predicted = (-coef[1] + np.sqrt(coef[1]**2 - 4*coef[2]*(intercept - X))) / (2*coef[2])

        return t_predicted


    def calc_h(self, aruco_corners):
        h = aruco_corners[0][0][3][1] - aruco_corners[0][0][1][1]
        return h

    def DetectTargetContinous(self):
        while cv2.waitKey(4) == -1: # Wait for a key pressed event

            try:
                print("start")
                image = self.cam.capture_array("main")
            
                cv2.imshow(self.WIN_RF, image)

                aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, self.arucoDict)
            
                h = self.calc_h(aruco_corners)
                arucoMarkerLength = self.Marker_length(h)
                intrinsic_matrix = self.intrinsic()
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, arucoMarkerLength, intrinsic_matrix, None)
                
                andersolddir = self.angle_between_vectors(np.array([tvecs[0][0][0],tvecs[0][0][2]]),np.array([0,1]))

                dist = np.linalg.norm(tvecs)
                enddist = self.predict_t_values((dist/100))
                print(f"enddist={enddist}")

            except:
                pass

    def CreateLandmarksMap(self):

        while cv2.waitKey(4) == -1: # Wait for a key pressed event
            print("1")
            KnownTvecs = []
            KnownIDs = []

            for _ in range(5):

                try:
                    image = self.cam.capture_array("main")
                
                    cv2.imshow(self.WIN_RF, image)
                    print("2")

                    aruco_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, self.arucoDict)

                    h = self.calc_h(aruco_corners)
                    arucoMarkerLength = self.Marker_length(h)
                    intrinsic_matrix = self.intrinsic()
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
        
    def EvaluateCollisionLandmarksMap(self, landmarksMap, pos):

        print(f"landmarksmap: {landmarksMap}")
        for elm,_ in landmarksMap:
            distance = np.sqrt((pos[0]-elm[0])**2+(pos[1]-elm[1])**2)
            print(f"Distance: {distance}")
            required_distance = 34 + 20 #obstacle radius + arlo-radius

            if (distance <= required_distance):
                return True

        return False


# testRoboCam = RoboCamera()
# 
# testRoboCam.CreateLandmarksMap()
# testRoboCam.DetectTargetContinous()
# testRoboCam.EvaluateCollisionLandmarksMap()