from time import sleep

import robot

arlo = robot.Robot()

print("Running...")

#print(arlo.go_diff(64,64, 1, 0))
#sleep(2)
while True:
    print("Front ping: " + str(arlo.read_front_ping_sensor()))

    # print("End run =)")
