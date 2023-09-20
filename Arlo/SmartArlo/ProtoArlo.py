from time import sleep

import robot

import numpy as np

import math

class ProtoArlo:

    def __init__(self):
        self.target = np.array([0,0])
        self.pos = np.array([0,0])
        self.dir = np.array([0,1])
        self.speed = 40
        self.arlo = robot.Robot()
    
    def SetSpeed(self, speed): # Obsolete
        #set speed
        pass

    def Drive(self, dist):
        #drive straigt for dist meters

        # min_speed = 31

        # diff = self.speed - min_speed #min speed is 31

        # mult = np.array([x/5 for x in range(1,5)]) * diff + min_speed

        # print(mult)

        # for m in mult:
        #     self.arlo.go_diff(m, m, 1, 1)
        #     sleep(0.04)

        self.arlo.go_diff(self.speed, self.speed, 1, 1)
        sleep(dist)

        # for m in np.flip(mult):
        #     self.arlo.go_diff(m, m, 1, 1)
        #     sleep(0.08)

    
    def Turn(self, angle):

        turn_time = abs(0.01285*angle - 0.03500)

        turn_speed = 45

        if (angle > 0):
            self.arlo.go_diff(turn_speed, turn_speed, 1, 0)
            sleep(turn_time)
        else:
            self.arlo.go_diff(turn_speed, turn_speed, 0, 1)
            sleep(turn_time)
        

        self.arlo.stop()

        sleep(0.5)

    
    def GotoPos(self, vec):
        
        # angle = math.atan2(vec[1], vec[0]) * 180 / math.pi
        angle = np.degrees(np.arccos(np.dot(vec, self.dir) / (np.linalg.norm(vec) * np.linalg.norm(self.dir))))

        self.dir = vec

        dist = np.sqrt(vec.dot(vec))

        self.Turn(angle)

        self.Drive(dist)

        self.pos += vec
     



proto_arlo = ProtoArlo()
# proto_arlo.Turn(-90)
# proto_arlo.Turn(-90)

# proto_arlo.Drive(2)
# proto_arlo.Turn(90)
# proto_arlo.Drive(2)
# proto_arlo.Turn(90)
# proto_arlo.Drive(10)
# proto_arlo.Turn(120)
proto_arlo.Drive(1)
# proto_arlo.Turn(120)
# proto_arlo.Drive(1)





# proto_arlo.GotoPos(np.array([0,0.2]))
# proto_arlo.GotoPos(np.array([-1,0]))
# proto_arlo.GotoPos(np.array([0,0])-proto_arlo.pos)


# proto_arlo.Turn(90)


# proto_arlo.Turn(-180)






# # send a go_diff command to drive forward
# leftSpeed = 64
# rightSpeed = 64
# print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

# # Wait a bit while robot moves forward
# sleep(2)

# # send a stop command
# print(arlo.stop())

# # Wait a bit before next command
# sleep(0.041)

# # send a go_diff command to drive backwards the same way we came from
# print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))

# # Wait a bit while robot moves backwards
# sleep(1)

# # send a stop command
# print(arlo.stop())

# # Wait a bit before next command
# sleep(0.041)



# # request to read Front sonar ping sensor
# print("Front sensor = ", arlo.read_front_ping_sensor())
# sleep(0.041)


# # request to read Back sonar ping sensor
# print("Back sensor = ", arlo.read_back_ping_sensor())
# sleep(0.041)

# # request to read Right sonar ping sensor
# print("Right sensor = ", arlo.read_right_ping_sensor())
# sleep(0.041)

# # request to read Left sonar ping sensor
# print("Left sensor = ", arlo.read_left_ping_sensor())
# sleep(0.041)



# # send a go_diff command to drive forward in a curve turning right
# leftSpeed = 64
# rightSpeed = 32
# print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

# # Wait a bit while robot moves forward
# sleep(3)

# # send a stop command
# print(arlo.stop())

# # Wait a bit before next command
# sleep(0.041)

# # send a go_diff command to drive backwards the same way we came from
# print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))

# # Wait a bit while robot moves backwards
# sleep(3)

# # send a stop command
# print(arlo.stop())



# print("Finished")
