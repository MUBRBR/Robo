

from ..lib.rrt import RRT
from ..lib.grid_occ import GridOccupancyMap


def main():

    # Map gets made here
    path_res = 0.05
    map = GridOccupancyMap(low=(0, 0), high=(3, 4), res=path_res)
    map.populate()
    # map.register_obstacle([np.array([2,2]),np.array([1.5,2]),np.array([1,3])])

    #RRT is initialized here
    rrt = RRT(
        start=[1.5,0.0],
        goal=[1.5,3.5],
        map=map,
        expand_dis=0.1,
        path_resolution=path_res
        )
    
    path, optimized_path = rrt.planning() # optimized should be used by robot
    print(f"Path:\n{path}\n")
    print(f"Optimized path:\n{optimized_path}\n")
    rrt.draw_graph(path, optimized_path) #path should be saved so it can be viewed afterwards

if __name__ == '__main__':
    main()