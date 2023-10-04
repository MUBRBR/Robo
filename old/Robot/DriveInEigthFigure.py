from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward
# Wait a bit while robot moves forward
leftSpeed = 57 # Left motor seems slightly weaker. Increased power for leftWheel to compensate.
rightSpeed = 56
turnSpeed = 1.90*2

for i in range(8):
    if (i % 2 == 0): # if even, left, else right
        leftSpeed = 127
        rightSpeed = 31
    else:
        leftSpeed = 31
        rightSpeed = 127

    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

    sleep(turnSpeed)

# send a stop command
print(arlo.stop())

