from time import sleep
import robot as robot
import numpy as np
import queue as q
import math
import os
import radar 
import camera
import particle_filter
import rrt
from grid_occ import GridOccupancyMap

os.system("clear && printf '\e[3J'")



class proto_arlo():
    def __init__(self, landmarks):

        self.currPos = Vec3(0.0, 0.0, 0.0)
        
        self.speed = 50

        self.currentRoute = q.Queue()

        self.state = "INIT_LOCALIZE"
        self.currLm = 1


        # self.landmarks = [[1, 0.0, 0.0],[2, 0.0, 300.0], [3, 400.0, 0.0], [4, 400.0, 300.0]]
        self.landmarks = {
            1: (0.0, 0.0),
            2: (0.0, 300.0),
            3: (400.0, 0.0),
            4: (400.0, 300.0)
        }

        self.next_landmark_target = 1 # so we can +=1 at some point

        # initialize helper classes

        self.arlo = robot.Robot()

        self.cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        
        self.particle_filter = particle_filter.ParticleFilter(self.landmarks, self.cam)

        self.Radar = radar.Radar(self.arlo)

        self.RRT = rrt.RRT(map = GridOccupancyMap(low=(0, 0), high=(4, 3), res=0.05), cam = self.cam)


    def __del__(self):
        #Clean-up capture thread
        self.cam.terminateCaptureThread()

        # safely shut down arlo
        self.arlo.__del__()

    # spin 360 degrees and find unique landmarks and perform MCL with Self_localization
    def observe360Degrees(self):
        # rotating 20 degrees 18 times (360 degrees) and storing landmarks seen
        iterations = 18 # should be 18 but it over drives by 45degrees, 15 iterations = 360degrees
        for _ in range(iterations):
            self.RotateAngle(np.deg2rad(20))
            sleep(0.5)
            print(f"iter{_}")
            # self.particle_filter.move_particles(0.0, 0.0, np.deg2rad(20))  # we shouldnt move particles when we spin 360 degrees
            
            
            colour = self.cam.get_next_frame()
            # Detect objects
            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            
            #Get all indices that is not a reoccurring objectID. 
            if not isinstance(objectIDs, type(None)): 
                valid_indices = [i for i in range(len(objectIDs)) if 
                    (i == 0 and objectIDs[i] in self.landmarks.keys()) or # First ID is always valid unless it is not in landmarks
                    (objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in self.landmarks.keys())] # If ID is new it is valid unless it is not in landmarks
                if not isinstance(valid_indices, type(None)):
                # if (len(valid_indices > 0)):

                    self.particle_filter.perform_MCL(int (150), self_localize= True)
                

    def init_localize(self):
        # while det cam ser er none  eller objectids indeholde ikke L! 
                # Detect objects
        colour = self.cam.get_next_frame()

        objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            #Get all indices that is not a reoccurring objectID. 
        while(isinstance(objectIDs, type(None)) or self.currLm not in objectIDs): 
            colour = self.cam.get_next_frame()

            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            self.Log(f"{objectIDs = }, {dists = }, {angles = }")
            betterArlo.RotateAngle(np.deg2rad(20))

            sleep(0.8)
        
        betterArlo.RotateAngle(-np.deg2rad(20))

        sleep(0.8)

        for objectID, dist, angle in zip(objectIDs, dists, angles):
            if (objectID == self.currLm):

                self.particle_filter.perform_MCL(150, self_localize = True, early_stopping = True)

                self.particle_filter.move_particles_forward(dist)

                self.particle_filter.add_uncertainty(0.0, 0.1) 

                self.DriveVector((dist, 0.0))
                
                self.incrementLM()

                break
               


                #NO CHANGE IN STATE
                # self.state = "LOCALIZE"
                
    def localize(self):   
        # Here we'll call self_localize()
        # Reset queue, perform MCL with *10 particles while rotating
        self.observe360Degrees()
        self.state = "GET_PATH"
        

    def boot_and_rally(self):
        betterArlo.RotateAngle(np.deg2rad(20))

        while True:
            self.Log("Main loop")
            if self.state == "INIT_LOCALIZE":
                self.init_localize()

            elif self.state == "LOCALIZE":
                self.localize()
                
            elif self.state == "GET_PATH":
                # Estimate pose and then perform RRT to get a route to curr LM
                self.currPos = self.particle_filter.estimate_pose()
                self.Log(f"Estimated current pose in GET_path: {self.currPos}")
                dest = self.landmarks[self.currLm]
                self.Log(f"Dest: {dest} | currLM: {self.currLm}")
                angleToTarget = self.CalcTheta_target(self.currPos, dest)
                self.Log(f"AngleToTarget LM: {np.degrees(angleToTarget)}")
                self.RotateAngle(angleToTarget)
                optimal_path = self.RRT.get_path(self.currLm, self.currPos, dest, draw_map= True)
                self.Log(f"           OPTIMAL PATH: {optimal_path}")

                # Here we'll call follow_path()
                #if not isinstance(optimal_path, type(None)):
                if (1+1==4):
                    self.Log("Found safe path", "y")
                    self.currentRoute = q.Queue()
                    for i in range(1, len(optimal_path)):
                        betterArlo.AddDest(optimal_path[i])
                    # end with updating the currLm
                else:
                    self.Log("Could not find safe path", "y")
                    self.currentRoute = q.Queue()
                    betterArlo.AddDest(self.landmarks[self.currLm]) 
                self.state = "FOLLOW_PATH"


            elif self.state == "FOLLOW_PATH":
                betterArlo.FollowRoute(False)

                self.incrementLM()

                self.state = "GET_PATH"

            elif self.state == "FINISHED":
                self.Log("ProtoArlo has completed the Rally!")
                return

            else:
                # Should never be matched, but I do not like not having a default
                self.Log(f"Unexpected state: {self.state}")
                self.state = "LOCALIZE"


    def DriveVector(self, vector): # drives forward at the length of the given Vector. Keeps track of pos from Vector
        
        self.currPos[0] += vector[0]
        self.currPos[1] += vector[1]

        
        length = np.linalg.norm(vector)

        self.Log(f"vector to drive: {vector} | normed vektor: {length}")
        n = 1
        for _ in range(n):
            colour = self.cam.get_next_frame()
            # Detect objects
            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            
            #Get all indices that is not a reoccurring objectID. 
            # self.particle_filter.perform_MCL(int (20 / n))
            self.particle_filter.move_particles(vector[0] / n, vector[1] / n, 0.0) # mover ikke particles rigtigt her?
            self.particle_filter.add_uncertainty(0.5 / n, 0.0) # This is for when MCL is used. Maybe divided by n??
            self.DriveLength(length/n)
            self.currPos = self.particle_filter.estimate_pose()
            print(f"currPos estimate in drive: {self.currPos}")




    def DriveLength(self, dist): #Goes for4ward for dist length, does not stop on its own, does not update any fields
        self.Log(f"I drive {dist} centimers")
        #makes centimeters to meters (Drives in meters)
        dist /= 100 
            
        self.arlo.go_diff(self.speed, self.speed, 1, 1)
        sleep(dist * 3)


    def Stop(self):

        self.arlo.stop()


    def RotateAngle(self, angle): #turns by angle, does not update any fields
        # OBS RotateAngle uses degrees
        turn_time = abs(0.01417*np.degrees(angle) + 0.01667)

        turn_speed = 40

        self.Log(f"I turn {angle} degrees")
        self.Log(f"Rotating angle: {np.degrees(angle)} | Turntime: {turn_time}")
        n = 1
        for _ in range(n):
            colour = self.cam.get_next_frame()
            # Detect objects
            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            
            #Get all indices that is not a reoccurring objectID. 
            if not isinstance(objectIDs, type(None)): 
                valid_indices = [i for i in range(len(objectIDs)) if 
                    (i == 0 and objectIDs[i] in self.landmarks.keys()) or # First ID is always valid unless it is not in landmarks
                    (objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in self.landmarks.keys())] # If ID is new it is valid unless it is not in landmarks
                if not isinstance(valid_indices, type(None)):
                    self.particle_filter.perform_MCL(int (20 / n))
                    self.particle_filter.move_particles(0.0, 0.0, np.degrees(angle) / n) 
                    self.particle_filter.add_uncertainty(0.0, 0.1 / n) # This is for when MCL is used. Maybe divided by n??
            
            if (angle < 0):
                self.arlo.go_diff(turn_speed, turn_speed, 1, 0)
                sleep(turn_time / n)
            else:
                self.arlo.go_diff(turn_speed, turn_speed, 0, 1)
                sleep(turn_time / n)

            self.currPos = self.particle_filter.estimate_pose()
            print(f"estimated pose while rotating: {self.currPos}")
            self.arlo.stop()

    def RotateTime(self,time):

        turn_speed = 40

        self.arlo.go_diff(turn_speed, turn_speed, 1, 0)

        sleep(time)

        self.arlo.stop()
    

    def AddDest(self, dest):

        self.currentRoute.put(dest)

    def CalcTheta_target(self, currPos, dest):
        return np.arctan2(dest[1] - currPos[1], dest[0] - currPos[0]) - currPos[2]
        
    
    def GoToDest(self, dest): # dest is a Vector

        step_length = 10 # længde den prøver at køre i skridt

        clear_path_length = step_length * 2.5


        distance = (dest[0] - self.currPos[0], dest[1] - self.currPos[1])
        print(f"distance we give to drivevector: {distance}")
        self.Log("Moving forward")

        
        # Calculating target angle as arctan2(y_2 - y_1, x_2 - x_1) - Theta (Robots current angle)
        theta_target = self.CalcTheta_target(self.currPos, dest)
        print(f"Theta: {np.degrees(theta_target)}")
        
        
        self.RotateAngle(theta_target)

        self.Log( "after rotate towards current target dest")

        # before we drive, check for obstacles
        self.Radar.Update() 

        print("Before IF1")
        print(f"radar.DistW: {self.Radar.DistW()} | clearPathLength: {clear_path_length}")
        if (self.Radar.DistW() < clear_path_length): # something is in the way, need to avoid it. So stop, look, drive around.

            self.Log("Encountered obstacle", "r")

            self.Stop()
            
            # Rotate back to the step before turning
            self.RotateAngle(-theta_target)

            # Drive backwards to plan new route
            print("Before IF2")

            if (self.Radar.backCameraSafe()):
                self.Log("No obstacle behind us, driving backwards 45cm")
                self.DriveLength(-45)
                self.currentRoute = q.Queue()
                self.Log(f"Reset Queue")
                self.state = "GET PATH"
                return
        print("After IF1")
        
        self.DriveVector(distance)

        self.Stop()
    
        self.Log("Arrived at current destination", "g")

            

    def FollowRoute(self, reset):
        
        while (not self.currentRoute.empty()):
            self.GoToDest(self.currentRoute.get())

        # if ("dsitance to target" < 40):
            
        #     self.Log(f"Arrived at current landmark: {self.curr}", "g")
            
        #     match self.next_landmark_target:
        #         case 4:
        #             self.next_landmark_target = 1
        #         case _:
        #             self.next_landmark_target += 1

        #     self.state = "GET_PATH"
            
        #     if ("is last landmark"):
        #         self.Log("ITS A CELEBRATION", "g")

    def incrementLM(self):
        if self.currLm == 1:
            self.currLm = 2
        elif self.currLm == 2:
            self.currLm = 4
        elif self.currLm == 4:
            self.currLm = 3            
        elif self.currLm == 3:
            self.currLm = 1            


    def Log(self, action: str, color = "None"):

        if (color == "g"):
            print('\033[1m', end='') #bold
            print('\033[92m', end='') #green
        elif (color == "y"):
            print('\033[1m', end='') #bold
            print('\033[93m', end='') #green
        elif (color == "r"):
            print('\033[1m', end='') #bold
            print('\033[91m', end='') #red

        # with np.printoptions(sign='+', floatmode="fixed", precision=1):

            
        pose = self.particle_filter.estimate_pose()
        print(f"POS X: {pose[0]}, POS Y: {pose[1]}, θ: {pose[2]}, State: {self.state}, LM = {self.currLm} -> " + action)
        # with np.printoptions(sign='+'):
            
        # # with np.printoptions(floatmode="fixed", precision=1):

        #     angle = angle_between_vectors([0,1], self.currDir)
            

            # print(f"POS: {self.currPos}, DIR: {self.currDir}, θ: {angle} -> " + action)
        
        print('\033[0m', end='')













def Vec(x,y):
    return np.array([x,y])

def Vec3(x,y,z):
    return np.array([x,y,z])

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




# test
# BetterArlo starts in (150, 0, pi)
# try/finally to properly stop the program when using 'CTRL+C' to terminate program
try:
    betterArlo = proto_arlo((1))
    betterArlo.boot_and_rally()
finally:
    betterArlo.__del__()
