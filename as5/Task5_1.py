

import cv2
# from ..lib.rrt import RRT
# from ..lib.grid_occ import GridOccupancyMap
# from ..ParticleFilter.camera import Camera
import camera as camera
# from ..ParticleFilter import camera as camera
# from ..ParticleFilter.framebuffer import *
import framebuffer
# from ..ParticleFilter.particle_filter import ParticleFilter as pf
# from ..lib.SmartArloNew import betterRobot as arlo
import SmartArloNew as arlo
import particle_filter as pf
from time import sleep



# landmarks to find
landmarkIDS1 = {
    1: (0.0, 300.0),
    2: (100.0, 300.0)
}
landmarkIDS2 = [(1, 0.0, 300.0), (2, 100.0, 300.0)]
# landmarkIDs = [(3, 0.0, 100.0), (4, 100.0, 100.0)] #tester

unique_indices = []

# Initialize particles
num_particles = 1000
particle_filter = pf.ParticleFilter([0,0],[1,1], landmarkIDS2, num_particles)

def main():
    try:
        cam = camera.Camera(0, 'arlo', useCaptureThread = True)
        
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)
        
        # Fetch next frame
        colour = cam.get_next_frame()
        cv2.imshow(WIN_RF1, colour)
        
        # Hardcoded 
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        unique_indices = []
        # makes unique landmarkIDs
        if not isinstance(objectIDs, type(None)): # if there is actually work to do..
            unique_indices = [i for i in range(len(objectIDs)) 
                                if i == 0 and objectIDs[i] in landmarkIDS1.keys() or objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in landmarkIDS1.keys()] 
        # print(f"Zero objects found: {unique_indices}")
        
        print(f"Maybe found an object or 2?: {unique_indices}")
        roboarlo = arlo.betterRobot()
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
                

                





            print(f"Measure of how sure we are of the current estimated pose: {particle_filter.evaluate_pose()}")

            if not isinstance(objectIDs, type(None)): # if there is actually work to do..

                particle_filter.MCL(objectIDs, dists, angles)

                particle_filter.add_uncertainty(0.5,0.1)

            else:
                # No observation - reset weights to uniform distribution
                particle_filter.reset_weights()

                particle_filter.add_uncertainty(1,0.1)

            est_pose = particle_filter.estimate_pose() # The estimate of the robots current pose
            print(f"Estimated position: {est_pose}")
    
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()

        # Clean-up capture thread
        cam.terminateCaptureThread()
        

if __name__ == '__main__':
    main()

    