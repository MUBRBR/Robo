import cv2 # Import the OpenCV library
import time
import math
import numpy as np
from pprint import *
# import SmartArloNew as arlo
from grid_occ import *
from RoboCamera import *
from rrt import *


roboCam = RoboCamera()


path_res = 0.05
map = GridOccupancyMap(low=(-3, 0), high=(3, 4), res=path_res)
print("create landsmarks",roboCam.CreateLandmarksMap())
map.register_obstacle(roboCam.CreateLandmarksMap())

rrt = RRT(
    start=[0.0,0.0],
    goal =[2.0,3.5],
    map = map,
    expand_dis=0.1,
    path_resolution=path_res
    )

path, optimal_path = rrt.planning()
rrt.draw_graph(path, optimal_path)
map.draw_map()


# map.in_collision([0,0])
# map.draw_map()




