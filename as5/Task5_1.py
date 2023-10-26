
import numpy as np
import cv2
# from ..lib.rrt import RRT
# from ..lib.grid_occ import GridOccupancyMap
# from ..ParticleFilter.camera import Camera
import camera as camera
# from ..ParticleFilter import camera as camera
# from ..ParticleFilter.framebuffer import *
import framebuffer
# from ..ParticleFilter.particle_filter import ParticleFilter as pf
from particle_filter_visualizer import draw_world 
# from ..lib.SmartArloNew import betterRobot as arlo
import SmartArloNew as arlo
import particle_filter as pf
from time import sleep



# landmarkIDs = [(3, 0.0, 100.0), (4, 100.0, 100.0)] #tester

# unique_indices = []
# landmarks to find
landmarkIDS1 = {
    1: (0.0, 300.0),
    2: (100.0, 300.0)
}
# double landmarks because bad code needs landmarks as dict + as list.
landmarkIDS2 = [(1, 0.0, 300.0), (2, 100.0, 300.0)]


def main():
    try:
        
        
        
        # create robo object
        roboarlo = arlo.betterRobot()
        
        # init windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)
        
        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)
        
        # init cam
        cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        
        # Fetch next frame
        colour = cam.get_next_frame()
        cv2.imshow(WIN_RF1, colour)
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        unique_indices = []
        
        # Initialize particles
        num_particles = 1000
        particle_filter = pf.ParticleFilter([0,0],[1,1], landmarkIDS2, num_particles)
        
        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)

        # Draw map
        draw_world(est_pose, particle_filter, world)
        
        # makes unique landmarkIDs
        if not isinstance(objectIDs, type(None)): # if there is actually work to do..
            unique_indices = [i for i in range(len(objectIDs)) 
                                if i == 0 and objectIDs[i] in landmarkIDS1.keys() or objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in landmarkIDS1.keys()] 
        
        # print(f"Zero objects found: {unique_indices}")
        
        print(f"Maybe found an object or 2?: {unique_indices}")
        
        while True:
            action = cv2.waitKey(10)
            colour = cam.get_next_frame()
            cv2.imshow(WIN_RF1, colour)
            
            
            if action == ord('q'): # Quit
                break
            
            while len(unique_indices) < 2: # indsæt timer så den begynder at køre nye steder for at lede efter tid
                roboarlo.RotateAngle(20)
                sleep(0.5)
                
                colour = cam.get_next_frame()
                cv2.imshow(WIN_RF1, colour)
                # Detect objects
                objectIDs, dists, angles = cam.detect_aruco_objects(colour)
                print(f"Objects in view: {objectIDs}")
                
                # makes unique landmarkIDs
                if not isinstance(objectIDs, type(None)): # if there is actually work to do..
                    unique_indices = [i for i in range(len(objectIDs)) 
                                    if i == 0 and objectIDs[i] in landmarkIDS1.keys() or objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in landmarkIDS1.keys()] 

                print(f"After rotate: {unique_indices}")
                

                





            # print(f"Measure of how sure we are of the current estimated pose: {particle_filter.evaluate_pose()}")

            if not isinstance(objectIDs, type(None)): # if there is actually work to do..

                particle_filter.MCL(objectIDs, dists, angles)

                particle_filter.add_uncertainty(0.5,0.1)

            else:
                # No observation - reset weights to uniform distribution
                particle_filter.reset_weights()

                particle_filter.add_uncertainty(1,0.1)

            est_pose = particle_filter.estimate_pose() # The estimate of the robots current pose
            # print(f"Estimated position: {est_pose}")
    
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()

        # Clean-up capture thread
        cam.terminateCaptureThread()
        

if __name__ == '__main__':
    main()

    