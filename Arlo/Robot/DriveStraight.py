from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward
# Wait a bit while robot moves forward
leftSpeed = 64
rightSpeed = 64
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
sleep(2.27)

print(arlo.stop())



# 56 speed + 2.68