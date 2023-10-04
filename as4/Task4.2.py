import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *
# import SmartArloNew as arlo
import sys

sys.path.insert(0, '..')
from lib.grid_occ import *
from lib.RoboCamera import *
from lib.rrt import *
from lib.SmartArloNew import betterRobot

roboCam = RoboCamera()


path_res = 0.05
map = GridOccupancyMap(low=(-3, 0), high=(3, 4), res=path_res)
map.register_obstacle(roboCam.CreateLandmarksMap())

rrt = RRT(
    start=[0.0,0.0],
    goal =[0.0,3.5],
    map = map,
    expand_dis=0.1,
    path_resolution=path_res
    )

path, optimal_path = rrt.planning()
rrt.draw_graph(path, optimal_path)
map.draw_map()

arlo = betterRobot()

arlo.initDir = np.array([1,0])
for i in range(1,len(optimal_path)):
    arlo.AddDest(optimal_path[i])
arlo.FollowRoute(False)
