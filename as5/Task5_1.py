

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
    3: (0.0, 100.0),
    8: (100.0, 100.0)
}
landmarkIDS2 = [(3, 0.0, 100.0), (8, 100.0, 100.0)]
# landmarkIDs = [(3, 0.0, 100.0), (4, 100.0, 100.0)] #tester

unique_indices = []

# Initialize particles
num_particles = 1000
particle_filter = pf.ParticleFilter([0,0],[1,1], landmarkIDS2, num_particles)

def main():
    cam = camera.Camera(0, 'arlo', useCaptureThread = True)

    # Fetch next frame
    colour = cam.get_next_frame()
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
        if action == ord('q'): # Quit
            break
        
        while len(unique_indices) < 1: # indsæt timer så den begynder at køre nye steder for at lede efter tid
            roboarlo.RotateAngle(20)
            sleep(1)
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


if __name__ == '__main__':
    print("abc")
    main()
    cv2.destroyAllWindows()
    cam.terminateCaptureThread()
    