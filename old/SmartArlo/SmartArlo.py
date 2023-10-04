from time import sleep

import robot

import numpy as np

import queue as q

import math

def Vec2(x,y):
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

class betterRobot():
    def __init__(self):
        self.arlo = robot.Robot()

        self.initPos = Vec2(0.0,0.0)
        self.currPos = Vec2(0.0,0.0)
        
        self.initDir = Vec2(0.0,1.0)
        self.currDir = Vec2(0.0,1.0)

        self.speed = 64

        self.destQ = q.Queue()

        self.Radar = Vec3(0.0, 0.0, 0.0) # Front distance, weighted distance, direction 


    def UpdateRadar(self):
        
        left_distance = self.arlo.read_left_ping_sensor()

        front_distance = self.arlo.read_front_ping_sensor()
        
        right_distance = self.arlo.read_right_ping_sensor()

        weighted_distance = min((left_distance)*2,front_distance,(right_distance)*2) # We need to know if something is close, we use the closest regardless of direction. We dont care as much about the side sensors

        direction = -1 if ((-left_distance + right_distance) < 0) else 1 # if positive is obstacle is to the left (ie. turn right)

        self.Radar = Vec3(front_distance/1000,weighted_distance/1000,direction)

        self.Radar = Vec3(2,2,direction) # KILL RADAR



    def DriveVector(self, vector): # drives forward at the length of the given vec2tor. Keeps track of pos from vec2tor
        
        self.currPos += vector
        
        length = np.linalg.norm(vector)

        self.DriveLength(length)

    def DriveLength(self, dist): #Goes for4ward for dist length, does not stop on its own, does not update any fields
        
        self.arlo.go_diff(self.speed, self.speed, 1, 1)
        
        sleep(dist*.4)


    def Stop(self):

        self.arlo.stop()

    def RotateVector(self, vector):  # Turns the robot according point in the direction of the given vec2tor. Keeps track of direction from vec2tor.
        
        # angle = np.degrees(np.arccos(np.dot(vector, self.currDir / (np.linalg.norm(vector) * np.linalg.norm(self.currDir)))))
        angle = angle_between_vectors(self.currDir, vector)

        self.currDir = vector

        if (abs(angle) > 0.0001):

            print(f"I rotate {angle} degrees, my dir is {self.currDir}")
            self.RotateAngle(angle)
        
    def RotateAngle(self, angle): #turns by angle, does not update any fields
        
        turn_time = abs(0.01417*angle + 0.01667)

        turn_speed = 40

        print(f"Arlo turns {angle} degrees")

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

    def GoToDest(self,dest): # dest is a vec2tor

        step_length = 0.1 # længde den prøver at køre i skridt

        clear_path_length = step_length * 4

        while (np.linalg.norm(dest - self.currPos) > step_length): # while there is still some way to go, go!

            print(f"Relative angle: {angle_between_vectors([0,1],self.currDir)}")

            path = dest - self.currPos

            step = (path / np.linalg.norm(path)) * step_length

            # Rotate towards target

            self.RotateVector(step)

            print(f"LOG after rotate towards self.currDir: {self.currDir}, self.currPos{self.currPos}")


            # before we drive, check for obstacles
            self.UpdateRadar() 


            if (self.Radar[1] > clear_path_length): # nothing in the way, no need to stop - drive or keep driving

                # print("NO OBSTACLE")
                self.DriveVector(step)

            else: # something is in the way, need to avoid it. So stop, look, drive araound.
                print("OBSTACLE")
                self.Stop()

                while (self.Radar[1] < clear_path_length):

                    theta = np.deg2rad(-90)

                    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])

                    self.RotateVector(np.dot(rot, self.currDir))
                    
                    
                    print(f"LOG before turn drive self.currDir: {self.currDir}, self.currPos{self.currPos}")


                    # self.RotateVector(self.currDir @ np.array([[0,1],[1,0]]))

                    self.UpdateRadar() #after turning, and before checking again, ping the radar once more
                
                print("I try to get past the obstacle")

                self.DriveVector((self.currDir / np.linalg.norm(self.currDir)))

                print(f"LOG after drive self.currDir: {self.currDir}, self.currPos{self.currPos}")

                self.Stop()

        self.RotateVector(dest - self.currPos)

        self.DriveVector(dest - self.currPos)

        self.Stop()
    
        print(f"ARRIVED AT {self.currPos}")

            
    def FollowRoute(self, reset):
        
        while (not self.destQ.empty()):
            self.GoToDest(self.destQ.get())

        if (reset):
            self.GoToDest(self.initPos)
            self.RotateVector(self.initDir)

        print(f"Ended at {self.currPos}")
        print(f"Final dir {self.currDir}")



betterArlo = betterRobot()

# betterArlo.DriveLength(2)

# betterArlo.RotateAngle(-10)
# betterArlo.DriveLength(2)

# betterArlo.DriveVector(Vec2(0,2))
# betterArlo.DriveVector(Vec2(0,-2))




# betterArlo.AddDest(Vec2(0,1))
# betterArlo.AddDest(Vec2(0.4,1))
# betterArlo.AddDest(Vec2(-1,0.5))


# betterArlo.Stop()

# betterArlo.AddDest(Vec2(0,1))
# betterArlo.AddDest(Vec2(-1,0))
# betterArlo.AddDest(Vec2(-1,1))


# betterArlo.RotateTime(3.85)
# sleep(5)
# betterArlo.RotateAngle(360)



# betterArlo.AddDest(Vec2(-1,0))


# betterArlo.RotateVector((-1,0)) # (0,1)

betterArlo.AddDest(Vec2(0,1))
betterArlo.AddDest(Vec2(4,0.8))
betterArlo.AddDest(Vec2(0,4))
# betterArlo.AddDest(Vec2(0,-4))



# betterArlo.AddDest(Vec2(1,-1))



# betterArlo.FollowRoute(True)

betterArlo.FollowRoute(True)


