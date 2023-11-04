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

os.system("clear && printf '\e[3J'")



class proto_arlo():
    def __init__(self, landmarks):
        self.arlo = robot.Robot()

        self.initPos = Vec(0.0,0.0)
        self.currPos = Vec(0.0,0.0)
        
        self.initDir = Vec(0.0,1.0)
        self.currDir = Vec(0.0,1.0)

        self.speed = 50

        self.curretRoute = q.Queue()



        self.landmarks = [[1, 0.0, 0.0],[2, 0.0, 300.0], [3, 400.0, 0.0], [4, 400.0, 300.0]]

        self.particle_filter = particle_filter

        self.next_landmark_target = 1 # so we can +=1 at some point

        # initialize helper classes

        self.Radar = radar.Radar(self.arlo)

        self.cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)

        self.RRT = rrt.RRT()

    def __del__(self):
        # safely shut down arlo
        self.arlo.__del__()

        #Clean-up capture thread
        self.cam.terminateCaptureThread()


    def boot_and_rally():
        #log something here

        while True:
            pass


    def DriveVector(self, vector): # drives forward at the length of the given Vector. Keeps track of pos from Vector
        
        self.currPos += vector
        
        length = np.linalg.norm(vector)

        self.DriveLength(length)

    def DriveLength(self, dist): #Goes for4ward for dist length, does not stop on its own, does not update any fields
        
        self.arlo.go_diff(self.speed, self.speed, 1, 1)
        
        sleep(dist*3)


    def Stop(self):

        self.arlo.stop()


    def RotateVector(self, vector):  # Turns the robot according point in the direction of the given Vector. Keeps track of direction from Vector.
        
        # angle = np.degrees(np.arccos(np.dot(vector, self.currDir / (np.linalg.norm(vector) * np.linalg.norm(self.currDir)))))
        angle = angle_between_vectors(self.currDir, vector)

        self.currDir = vector

        if (abs(angle) > 0.0001):

            self.Log(f"I rotate {angle} degrees")
            self.RotateAngle(angle)
        
    def RotateAngle(self, angle): #turns by angle, does not update any fields
        
        turn_time = abs(0.01417*angle + 0.01667)

        turn_speed = 40

        self.Log(f"I turn {angle} degrees")

        if (angle > 0):
            self.arlo.go_diff(turn_speed, turn_speed, 1, 0)
            sleep(turn_time)
        else:
            self.arlo.go_diff(turn_speed, turn_speed, 0, 1)
            sleep(turn_time)

        self.arlo.stop()

    def RotateTime(self,time):

        turn_speed = 40

        self.arlo.go_diff(turn_speed, turn_speed, 1, 0)

        sleep(time)

        self.arlo.stop()
    

    def AddDest(self, dest):

        self.destQ.put(dest)


    def GoToDest(self,dest): # dest is a Vector

        step_length = 0.1 # længde den prøver at køre i skridt

        clear_path_length = step_length * 2.5

        while (np.linalg.norm(dest - self.currPos) > step_length): # while there is still some way to go, go!

            self.Log("Moving forward")

            path = dest - self.currPos

            step = (path / np.linalg.norm(path)) * step_length

            # Rotate towards target

            self.RotateVector(step)
            
            self.Log( "after rotate towards current target dest")

            # before we drive, check for obstacles
            self.Radar.Update() 


            if (self.Radar.DistW() < clear_path_length): # something is in the way, need to avoid it. So stop, look, drive araound.

                self.Log("Encountered obstacle", "r")

                self.Stop()

                while (self.Radar.DistW() < clear_path_length):

                    theta = np.deg2rad(-45)*self.Radar.obstacle_angle

                    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
                    
                    self.Log("before turn to avoid, before drive")

                    self.RotateVector(np.dot(rot, self.currDir))
                    
                    self.Log("after turn to avoid, before drive")

                    # self.RotateVector(self.currDir @ np.array([[0,-1],[1,0]]))

                    self.Radar.Update()  #after turning, and before checking again, ping the radar once more

                self.Log("before driving to get around obstacle")

                self.Log(f"Drive vect {(self.currDir / np.linalg.norm(self.currDir))*2}")

                self.DriveVector((self.currDir / np.linalg.norm(self.currDir))*2) 

                self.Log("after driving to get around obstacle")

                self.Stop()

            else: # nothing in the way, no need to stop - drive or keep driving

                self.DriveVector(step)

        self.RotateVector(dest - self.currPos)

        self.DriveVector(dest - self.currPos)

        self.Stop()
    
        self.Log("Arrived at current destination", "g")

            
    def FollowRoute(self, reset):
        
        while (not self.destQ.empty()):
            self.GoToDest(self.destQ.get())

        if (reset):
            self.GoToDest(self.initPos)
            self.RotateVector(self.initDir)

        self.Log("Arrived at final destination", "g")


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

        angle = angle_between_vectors([0,1], self.currDir)
            

        print(f"POS: {self.currPos}, DIR: {self.currDir}, θ: {angle} -> " + action)
        # with np.printoptions(sign='+'):
            
        # # with np.printoptions(floatmode="fixed", precision=1):

        #     angle = angle_between_vectors([0,1], self.currDir)
            

        #     print(f"POS: {self.currPos}, DIR: {self.currDir}, θ: {angle} -> " + action)
        
        print('\033[0m', end='')

betterArlo = proto_arlo()












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
