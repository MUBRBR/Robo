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
        self.arlo = robot.Robot()

        # self.initPos = Vec(0.0,0.0, 0.0) # not used
        self.currPos = Vec3(0.0, 0.0, 0.0)
        
        # self.initDir = Vec(0.0,1.0)
        # self.currDir = Vec(0.0,1.0)

        self.speed = 50

        self.cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        self.currentRoute = q.Queue()

        self.state = "INIT_LOCALIZE"



        # self.landmarks = [[1, 0.0, 0.0],[2, 0.0, 300.0], [3, 400.0, 0.0], [4, 400.0, 300.0]]
        self.landmarks = {
            1: (0.0, 0.0),
            2: (0.0, 300.0),
            3: (400.0, 0.0),
            4: (400.0, 300.0)
        }
        self.particle_filter = particle_filter.ParticleFilter(self.landmarks, self.cam)

        self.next_landmark_target = 1 # so we can +=1 at some point

        # initialize helper classes

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

                    self.particle_filter.perform_MCL(int (200), self_localize= True)
                



    def init_localize(self):
        # while det cam ser er none  eller objectids indeholde ikke L! 
                # Detect objects
        colour = self.cam.get_next_frame()

        objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)
            #Get all indices that is not a reoccurring objectID. 
        while(isinstance(objectIDs, type(None)) or 1 not in objectIDs): 
            colour = self.cam.get_next_frame()

            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)

            betterArlo.RotateAngle(np.deg2rad(20))

            sleep(0.5)
        
        for objectID, dist, angle in zip(objectIDs, dists, angles):
            if (objectID == 1):

                betterArlo.RotateAngle(angle)
                # print(f"{np.degrees(angle)}")
                self.Log("Rotating towards target LM1 with degrees: ")
                self.particle_filter.perform_MCL(750, self_localize = True, early_stopping = True)

                self.particle_filter.move_particles_forward(dist-50)
                self.particle_filter.add_uncertainty(0.0, 0.1) # This is for when MCL is used. Maybe divided by n??
        #                kør mod l1
                self.DriveVector((dist, 0.0))
                self.state = "LOCALIZE"
                
    #      
            

    def boot_and_rally(self):
        currLm = 1
        while True:
            print(f"\n\nState: {self.state}")
            if self.state == "LOCALIZE":
                # Here we'll call self_localize()
                # Reset queue, perform MCL with *10 particles while rotating
                self.currentRoute = q.Queue()
                self.observe360Degrees()
                self.state = "GET_PATH"
            
            elif self.state == "INIT_LOCALIZE":
                self.init_localize()
                currLm += 1
                
            elif self.state == "GET_PATH":
                # Estimate pose and then perform RRT to get a route to curr LM
                self.currPos = self.particle_filter.estimate_pose()
                print(f"Estimated current pose in GET_path: {self.currPos}")
                dest = self.landmarks[currLm]
                print(f"Dest: {dest} | currLM: {currLm}")
                angleToTarget = self.CalcTheta_target(self.currPos, dest)
                print(f"AngleToTarget LM: {np.degrees(angleToTarget)}")
                self.RotateAngle(angleToTarget)
                optimal_path = self.RRT.get_path(currLm, self.currPos, dest, draw_map= True)
                print(f"           OPTIMAL PATH: {optimal_path}")
                self.state = "FOLLOW_PATH"

            elif self.state == "FOLLOW_PATH":
                # Here we'll call follow_path()
                if not isinstance(optimal_path, type(None)):
                    for i in range(1, len(optimal_path)):
                        betterArlo.AddDest(optimal_path[i])
                    betterArlo.FollowRoute(1)
                    # end with updating the currLm
                if currLm != 4:
                    currLm += 1
                else:
                    currLm = 1
                self.state = "GET_PATH"

            elif self.state == "FINISHED":
                self.Log("ProtoArlo has completed the Rally!")
                return

            else:
                # Should never be matched, but I do not like not having a default
                self.Log(f"Unexpected state: {self.state}")
                self.state = "LOCALIZE"
            # match self.state:
                # case "LOCALIZE":
                    
                # # Here we''l call self_localize()
                # #   Reset queue, perform MCL with *10 parti while rotating
                #     self.currentRoute = q.Queue()
                #     self.observe360Degrees()
                #     self.state = "GET_PATH"

                # case "GET_PATH":
                #     # Estimate pose and then perform RRT to get a route to curr LM
                #     self.currPos = self.particle_filter.estimate_pose()
                #     dest = self.landmarks[currLm]
                #     angleToTarget = self.CalcTheta_target(self.currPos, dest)
                #     self.RotateAngle(angleToTarget)
                #     optimal_path = self.RRT.get_path(currLm, self.currPos, dest, draw_map= False)
                #     self.state = "FOLLOW_PATH" 

                # case "FOLLOW_PATH":
                #     # Here we'll call follow_path()
                #     for i in range(1,len(optimal_path)):
                #         betterArlo.AddDest(optimal_path[i])
                #     betterArlo.FollowRoute(1)
                #     # end with updating the currLm
                #     if currLm != 4:
                #         currLm += 1
                #     else:
                #         currLm = 1
                    

                # case "FINISHED":
                #     self.Log("ProtoArlo has completed the Rally!")
                #     return                                
                                        
                # case _: # Should never be matched, but I do not like not having a default
                #     self.Log(f"Unexpected state: {self.state = }")
                #     self.state = "LOCALIZE"



    def DriveVector(self, vector): # drives forward at the length of the given Vector. Keeps track of pos from Vector
        
        self.currPos[0] += vector[0]
        self.currPos[1] += vector[1]

        
        length = np.linalg.norm(vector)

        print(f"vector to drive: {vector} | normed vektor: {length}")
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


    # def RotateVector(self, vector):  # Turns the robot according point in the direction of the given Vector. Keeps track of direction from Vector.
        
    #     # angle = np.degrees(np.arccos(np.dot(vector, self.currDir / (np.linalg.norm(vector) * np.linalg.norm(self.currDir)))))
    #     angle = angle_between_vectors(self.currDir, vector)

    #     self.currDir = vector

    #     if (abs(angle) > 0.0001):

    #         self.Log(f"I rotate {angle} degrees")
    #         self.RotateAngle(angle)
        
    def RotateAngle(self, angle): #turns by angle, does not update any fields
        # OBS RotateAngle uses degrees
        turn_time = abs(0.01417*np.degrees(angle) + 0.01667)

        turn_speed = 40

        self.Log(f"I turn {angle} degrees")
        print(f"Rotating angle: {np.degrees(angle)} | Turntime: {turn_time}")
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
        
            # TO DO:
            # Implement an 'else'

            # self.Log(f"Drive vect {(self.currDir / np.linalg.norm(self.currDir))*2}")

            # self.DriveVector((self.currDir / np.linalg.norm(self.currDir))*2) 

            # self.Log("after driving to get around obstacle")

            # self.Stop()

        # else: # nothing in the way, no need to stop - drive or keep driving

        #     self.DriveVector(step)

        # self.RotateVector(dest - self.currPos)

        # self.DriveVector(dest - self.currPos)
        
        self.DriveVector(distance)
        # self.DriveLength(norm_distance)

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
        print(f"POS X: {pose[0]}, POS Y: {pose[1]}, θ: {pose[2]}, State: {self.state} -> " + action)
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
