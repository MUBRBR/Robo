from time import sleep
import robot as robot
import numpy as np
import queue as q
import math
import os
import radar 
import camera
import particle_filter
# import rrt

os.system("clear && printf '\e[3J'")



class proto_arlo():
    def __init__(self, landmarks):
        self.arlo = robot.Robot()

        self.initPos = Vec(0.0,0.0)
        self.currPos = Vec(0.0,0.0)
        
        # self.initDir = Vec(0.0,1.0)
        # self.currDir = Vec(0.0,1.0)

        self.speed = 50

        self.cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        self.currentRoute = q.Queue()

        self.state = "LOCALIZE"



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


        # self.RRT = rrt.RRT()

        self.est_pose = np.array([150.0, 0.0, np.pi])

    def __del__(self):
        #Clean-up capture thread
        self.cam.terminateCaptureThread()

        # safely shut down arlo
        self.arlo.__del__()


 
    # def boot_and_rally(self):
        
        # while True:
            
        #     match self.state:
        #         case "LOCALIZE":
        #             pass
        #         # Here we''l call self_localize()

        #         case "GET_PATH":
        #             pass

        #         case "FOLLOW_PATH":
        #             pass
        #             # Here we''l call follow_path()

        #         case "FINISHED":
        #             self.Log("ProtoArlo has completed the Rally!")
        #             return                                
                                        
        #         case _: # Should never be matched, but I do not like not having a default
        #             self.Log(f"Unexpected state: {self.state = }")
        #             self.state = "LOCALIZE"


    # def DriveVector(self, vector): # drives forward at the length of the given Vector. Keeps track of pos from Vector
        
    #     self.currPos += vector
        
    #     length = np.linalg.norm(vector)

    #     self.DriveLength(length)

    def DriveLength(self, dist): #Goes for4ward for dist length, does not stop on its own, does not update any fields
        self.Log(f"I drive {dist} centimers")
        #makes centimeters to meters (Drives in meters)
        dist /= 100
        n = 2
        for _ in range(n):
            self.particle_filter.move_particles(dist[0] / n, dist[1] / n , 0.0)
            # self.particle_filter.perform_MCL()
            # self.particle_filter.ParticleFilter.add_uncertainty(0.5 / n, 0.0) # This is for when MCL is used. Maybe divided by n??


            self.arlo.go_diff(self.speed, self.speed, 1, 1)
            
            sleep((dist * 3) / n)


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

        n = 2
        for _ in range(n):
            self.particle_filter.ParticleFilter.move_particles(0.0, 0.0, np.degrees(angle) / n)
            # self.particle_filter.perform_MCL()
            # self.particle_filter.ParticleFilter.add_uncertainty(0.0, 0.1 / n) # This is for when MCL is used. Maybe divided by n??

            if (angle < 0):
                self.arlo.go_diff(turn_speed, turn_speed, 1, 0)
                sleep(turn_time / n)
            else:
                self.arlo.go_diff(turn_speed, turn_speed, 0, 1)
                sleep(turn_time / n)

        self.arlo.stop()

    def RotateTime(self,time):

        turn_speed = 40

        self.arlo.go_diff(turn_speed, turn_speed, 1, 0)

        sleep(time)

        self.arlo.stop()
    

    def AddDest(self, dest):

        self.currentRoute.put(dest)


    def GoToDest(self,dest): # dest is a Vector

        step_length = 0.1 # længde den prøver at køre i skridt

        clear_path_length = step_length * 2.5


        norm_distance = np.linalg.norm(dest[0] - self.est_pose[0], dest[1] - self.est_pose[1])

        # while (norm_distance > step_length): # while there is still some way to go, go!
        # while (np.linalg.norm(dest - self.currPos) > step_length): # while there is still some way to go, go!

        self.Log("Moving forward")

        # path = dest - self.currPos
        path = (dest[0] - self.est_pose[0], dest[1] - self.est_pose[1])

        # Calculating target angle as arctan2(y_2 - y_1, x_2 - x_1) - Theta (Robots current angle)
        theta_target = np.arctan2(dest[1] - self.est_pose[1], dest[0] - self.est_pose[0]) - self.est_pose[2]

        
        
        self.RotateAngle(theta_target)


        # angle = angle_between_vectors(dest, (self.est_pose[0], self.est_pose[1]))

        # step = (path / np.linalg.norm(path)) * step_length

        # Rotate towards target

        # self.RotateVector(step)
        


        self.Log( "after rotate towards current target dest")

        # before we drive, check for obstacles
        self.Radar.Update() 


        if (self.Radar.DistW() < clear_path_length): # something is in the way, need to avoid it. So stop, look, drive around.

            self.Log("Encountered obstacle", "r")

            self.Stop()
            
            # Rotate back to the step before turning
            self.RotateAngle(-theta_target)

            # Drive backwards to plan new route
            if (self.Radar.backCameraSafe()):
                self.Log("No obstacle behind us, driving backwards 45cm")
                self.DriveLength(-45)
                self.currentRoute = q.Queue()
                self.Log(f"Reset Queue")
                self.state = "GET PATH"
                return
            
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

        self.DriveLength(norm_distance)

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

        # angle = angle_between_vectors([0,1], self.currDir)
            

        # print(f"POS: {self.currPos}, DIR: {self.currDir}, θ: {angle} -> " + action)
        # with np.printoptions(sign='+'):
            
        # # with np.printoptions(floatmode="fixed", precision=1):

        #     angle = angle_between_vectors([0,1], self.currDir)
            

        #     print(f"POS: {self.currPos}, DIR: {self.currDir}, θ: {angle} -> " + action)
        
        print('\033[0m', end='')













def Vec(x,y):
    return np.array([x,y])

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
betterArlo = proto_arlo((1))
betterArlo.AddDest((0,100))
betterArlo.AddDest((200,150))
betterArlo.AddDest((400,200))
betterArlo.FollowRoute(1)


