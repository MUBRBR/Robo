from time import sleep

import robot

import numpy as np


class ProtoArlo:

    def __init__(self):
        self.target = np.array([0,0])
        self.pos = np.array([0,0])
        self.speed = 56
        self.arlo = robot.Robot()
    
    def SetSpeed(self, speed):
        #set speed
        pass

    def Drive(self, dist):
        #drive straigt for dist meters

        min_speed = 31

        diff = self.speed - min_speed #min speed is 31

        mult = np.array([x/5 for x in range(1,5)]) * diff + min_speed

        print(mult)

        for m in mult:
            self.arlo.go_diff(m, m, 1, 1)
            sleep(0.04)

        self.arlo.go_diff(self.speed, self.speed, 1, 1)
        sleep(1)

        for m in mult.reverse():
            self.arlo.go_diff(m, m, 1, 1)
            sleep(0.08)

        self.arlo.stop()
    
    def Turn(self, angle):
        
        turn_time = 1

        if (angle > 0):
            self.arlo.go_diff(leftSpeed, rightSpeed, 1, 0)
            sleep(turn_time)
        else:
            self.arlo.go_diff(leftSpeed, rightSpeed, 0, 1)
            sleep(turn_time)
        
        self.arlo.stop()

    
    


proto_arlo = ProtoArlo()
proto_arlo.Drive(1)





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
