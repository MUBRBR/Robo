"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
import matplotlib.pyplot as plt

class GridOccupancyMap(object):
    """

    """
    def __init__(self, low=(0, 0), high=(2, 2), res=0.05) -> None:
        self.map_area = [low, high]    #a rectangular area    
        self.map_size = np.array([high[0]-low[0], high[1]-low[1]])
        self.resolution = res

        self.n_grids = [ int(s//res) for s in self.map_size]

        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]), dtype=np.uint8)

        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    def in_collision(self, pos):
        ####
        #### must return box number as val?
        ####
        indices = [int((pos[i] - self.map_area[0][i]) // self.resolution) for i in range(2)]
        for i, ind in enumerate(indices):
            if ind < 0 or ind >= self.n_grids[i]:
                return 1
        
        return self.grid[indices[0], indices[1]] 

    def populate(self, n_obs=6): # only really needed for testing rrt algorithm
        """
        generate a grid map with some circle shaped obstacles
        """
        origins = np.random.uniform(
            low=self.map_area[0] + self.map_size[0]*0.2, 
            high=self.map_area[0] + self.map_size[0]*0.8, 
            size=(n_obs, 2))
        radius = np.random.uniform(low=0.1, high=0.3, size=n_obs)
        #fill the grids by checking if the grid centroid is in any of the circle
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array([self.map_area[0][0] + self.resolution * (i+0.5), 
                                     self.map_area[0][1] + self.resolution * (j+0.5)])
                for o, r in zip(origins, radius):
                    if np.linalg.norm(centroid - o) <= r:
                        self.grid[i, j] = 1
                        break

    def register_obstacle(self,posLst,radius=0.56): #raidus provided in meters 
        ####
        #### must register box number in grid
        ####
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array([self.map_area[0][0] + self.resolution * (i+0.5), 
                                     self.map_area[0][1] + self.resolution * (j+0.5)])
                for pos in posLst:
                    if np.linalg.norm(centroid - pos) <= radius:
                        self.grid[i, j] = 1
                    

    def draw_map(self):
        #note the x-y axes difference between imshow and plot
        plt.clf()
        plt.grid()
        plt.imshow(self.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=1, extent=self.extent, interpolation='none')
        plt.show()

if __name__ == '__main__':
    path_res = 0.05
    map = GridOccupancyMap(low=(0, 0), high=(3, 4), res=path_res)
    # map.populate()
    map.register_obstacle([np.array([1,1]),np.array([1,2])])
    map.draw_map()
    plt.show()