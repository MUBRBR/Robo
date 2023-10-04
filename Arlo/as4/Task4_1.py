import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *
# import SmartArloNew as arlo
from grid_occ import *
from RoboCamera import *


roboCam = RoboCamera()


testPos = [1,0]
path_res = 0.05
map = GridOccupancyMap(low=(-3, 0), high=(3, 4), res=path_res)
print("create landsmarks",roboCam.CreateLandmarksMap())
map.register_obstacle(roboCam.CreateLandmarksMap(),radius=0.34)
map.in_collision([0,0])
map.draw_map()




