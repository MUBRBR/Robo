import numpy as np
import matplotlib.pyplot as plt
from camera import Camera
from grid_occ import GridOccupancyMap
import math

def polar_to_cartesian(magnitude, angle_in_radians):
    # Calculate the horizontal and vertical components
    x = magnitude * math.cos(angle_in_radians)
    y = magnitude * math.sin(angle_in_radians)
    
    # Return a tuple (x, y) representing the vector components
    return x, y


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, pos):
            self.pos = pos      #configuration position, usually 2D/3D for planar robots  
            self.path = []      #the path with a integration horizon. this could be just a straight line for holonomic system
            self.parent = None
        
        def calc_distance_to(self, to_node):
            # node distance can be nontrivial as some form of cost-to-go function for e.g. underactuated system
            # use euclidean norm for basic holonomic point mass or as heuristics
            d = np.linalg.norm(np.array(to_node.pos) - np.array(self.pos))
            return d
        
    def __init__(self,
                 map,           
                 cam,
                 start=[0.0,0.0],
                 goal=[0.0,0.0],                 
                 expand_dis=0.1,
                 path_resolution=0.05,
                 goal_sample_rate=5,
                 max_iter=500,
                 ):

        self.start = self.Node(np.array(start))
        self.end = self.Node(np.array(goal))
        self.map = map
        
        self.min_rand = map.map_area[0]
        self.max_rand = map.map_area[1]

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

        self.node_list = []
        self.edges = []

        self.cam = cam

    def planning(self):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]

        for _ in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision_free(new_node):
                self.node_list.append(new_node)

            #try to steer towards the goal if we are already close enough
            if self.node_list[-1].calc_distance_to(self.end) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision_free(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path
    
    def forward_dyn(self, x, u, T):
        path = [x]
        #note u must have T ctrl to apply
        for i in range(T):
            x_new = path[-1] + u[i] #u is velocity command here
            path.append(x_new)    
        
        return path[1:]    
    
    def inverse_dyn(self, x, x_goal, T):
        #for point mass, the path is just a straight line by taking full ctrl_range at each step
        dir = (x_goal-x)/np.linalg.norm(x_goal-x)

        u = np.array([dir*0.05 for _ in range(T)])

        return self.forward_dyn(x, u, T)

    def steer(self, from_node, to_node, extend_length=float("inf")):
        # integrate the robot dynamics towards the sampled position
        # for holonomic point pass robot, this could be straight forward as a straight line in Euclidean space
        # while need some local optimization to find the dynamically closest path otherwise
        new_node = self.Node(from_node.pos)
        d = new_node.calc_distance_to(to_node)

        new_node.path = [new_node.pos]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length // self.path_resolution)

        if n_expand > 0:
            steer_path = self.inverse_dyn(new_node.pos, to_node.pos, n_expand)

            #use the end position to represent the current node and update the path
            new_node.pos = steer_path[-1]
            new_node.path += steer_path

        d = new_node.calc_distance_to(to_node)
        if d <= self.path_resolution:
            #this is considered as connectable
            new_node.path.append(to_node.pos)

            #so this position becomes the representation of this node
            new_node.pos = to_node.pos.copy()

        new_node.parent = from_node
        if not (self.map.in_collision(np.array(new_node.pos))):
        # if  (self.map.in_collision(np.array(new_node.pos))):

            self.edges += [[new_node.pos,from_node.pos]] #Add edges for plotting

        return new_node

    def path_free_of_collision(self, start_pos, end_pos): 

        # Calculate the direction vector
        direction = np.array(end_pos) -  np.array(start_pos)

        # Calculate distance and steps between points
        distance = np.linalg.norm(direction)
        num_steps = int(distance / (self.expand_dis/2))
        step_size = direction / num_steps

        # Iterate over the line
        for i in range(num_steps + 1):
            current_point =  np.array(start_pos) + i * step_size

            if self.map.in_collision(current_point):
                return False

        return True

    def optimize_path(self, head, tail):    
        if (len(tail) == 0): # Base case, if theres only one value left that must be the dest (f#-esque solution)
            return [head]
        
        if (self.path_free_of_collision(head,tail[-1])): #If we can make it from the current node to the goal in a stright shot, do that.
            return [head] + [tail[-1]]

        best_index = 0 # We'll look for the next node in the path were the path connecting that to head in unobstructed
        for i in range(1,len(tail)):
            if (self.path_free_of_collision(head,tail[i])):
                best_index = i
            else:
                break 

        newHead = tail[best_index] #Shifting the head to the next "good" node skips all the unnecesary nodes
        newTail = tail[best_index+1:] if (len(tail) > best_index) else [] # This is again f# implementation, but it works!
        return [head] + self.optimize_path(newHead,newTail) #Heads are part of the new path, tails are ignored

    def generate_final_course(self, goal_ind):
        path = [self.end.pos]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.pos)
            node = node.parent
        path.append(node.pos)

        path = path[::-1]

        optimized_path = self.optimize_path(path[0],path[1:])

        return path, optimized_path

    def get_random_node(self):
        if np.random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                np.random.uniform(self.map.map_area[0], self.map.map_area[1])
                )
        else:  # goal point sampling
            rnd = self.Node(self.end.pos)
        return rnd

    def draw_graph(self, path, optimized_path, file_name = "Found path"):
        plt.clf()
        self.map.draw_map()
        for edge in self.edges:
            # Convert lists to numpy arrays for consistency
            edge = [np.array(p) for p in edge]

            # Extract x and y coordinates for each point
            x_coords, y_coords = zip(*edge)

            # Plot the line
            plt.plot(x_coords, y_coords, '-g')
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.plot([x for (x, y) in optimized_path], [y for (x, y) in optimized_path], '-b')
        plt.grid(True)
        plt.savefig(f"{file_name}.png", bbox_inches='tight')

    def check_collision_free(self, node):
        if node is None:
            return False
        for p in node.path:
            if self.map.in_collision(np.array(p)):
                return False
        return True
    
    def get_path(self, current_landmark, current_pose, target, draw_map = True):
        
        pos = np.array(current_pose)[:2]/100
        goal = np.array([abs(target[0]-24),abs(target[1]-18)])/100 #subtracting 24 and 18 shifts the target destination 30 cm towards the center of the course
        theta = current_pose[2]

        print(f"{pos = }, {goal = }, {theta =}")

        # Fist set start and goal (current position, goal)
        self.start = self.Node(pos)
        self.end = self.Node(goal)

        #Get frame and estblish position of obstacles
        colour = self.cam.get_next_frame()
        IDs, dists, angles = self.cam.detect_aruco_objects(colour)

        if isinstance(IDs, type(None)): #should never happen, but at least we can check
            return None

        for id, dist, angle in zip(IDs, dists,angles):
            new_vector = np.array(polar_to_cartesian(dist/100,angle+theta)) + pos
            print(f"{id} found: {new_vector}, relative angle: {angle}")

            if (id == current_landmark): # if its our current target don't block it
                break
            elif (id in [1,2,3,4]): # if its a landmark we need to go oruond in a wider radius
                self.map.register_obstacle(new_vector, radius = .50)
            else: #else its just an obstacle
                self.map.register_obstacle(new_vector, radius = .30)
                
        path, optimized_path = self.planning() # optimized should be used by robot

        # if we want to, draw
        if (draw_map):
            self.map.draw_map()
            # plt.show()
            
            # print(f"Path:\n{path}\n")
            # print(f"Optimized path:\n{optimized_path}\n")
            self.draw_graph(path, optimized_path)

        return [pos * 100 for pos in optimized_path] # back to using cm as unit


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [ node.calc_distance_to(rnd_node)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

def main():

    # just needs to be pasted into arlo

    # Map gets made here
    map = GridOccupancyMap(low=(0, 0), high=(4, 3), res=0.05)

    camera = Camera(0, 'macbookpro', useCaptureThread = False)

    #RRT is initialized here
    rrt = RRT(
        map=map,
        cam = camera,
        )
    
    print(f"{rrt.get_path(2,[0.0,0.0,.25*math.pi], [400.0,0.0]) = }")


if __name__ == '__main__':
    main()
