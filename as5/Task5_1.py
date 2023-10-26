
import numpy as np
import cv2
import camera as camera
import framebuffer
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
# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particle_filter, world):

    landmark_colors = [CRED, CGREEN, CBLACK, CBLUE]

    particles = particle_filter.particles

    offsetX = 100
    offsetY = 250
    ymax = world.shape[0]

    world[:] = CWHITE

    max_weight = np.max(particles[:, 3])

    # Draw particles
    particle_positions = particles[:, :2] + np.array([offsetX, offsetY])
    particle_angles = particles[:, 2]

    x = particle_positions[:, 0].astype(int)
    y = ymax - particle_positions[:, 1].astype(int)
    colours = np.array([jet(x) for x in particles[:, 3] / max_weight])

    particle_ends = particle_positions[:, :2] + 15.0 * np.column_stack((np.cos(particle_angles), -np.sin(particle_angles)))

    for i in range(len(particles)):
        cv2.circle(world, (x[i], y[i]), 2, colours[i], 2)
        b = (int(particle_ends[i, 0]), int(ymax - particle_ends[i, 1]))
        cv2.line(world, (x[i], y[i]), b, colours[i], 2)

    # Draw landmarks
    for i, (l_x,l_y) in enumerate(particle_filter.landmarks.values()):
        lm = (int(l_x + offsetX), int(ymax - (l_y + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

        

    # Draw estimated robot pose
    a = (int(est_pose[0]) + offsetX, ymax - (int(est_pose[1]) + offsetY))
    b = (int(est_pose[0] + 15.0 * np.cos(est_pose[2])) + offsetX, ymax - (int(est_pose[1] + 15.0 * np.sin(est_pose[2])) + offsetY))

    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)

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

        
        
        # makes unique landmarkIDs
        if not isinstance(objectIDs, type(None)): # if there is actually work to do..
            unique_indices = [i for i in range(len(objectIDs)) 
                                if i == 0 and objectIDs[i] in landmarkIDS1.keys() or objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in landmarkIDS1.keys()] 
        
        # print(f"Zero objects found: {unique_indices}")
        
        print(f"Maybe found an object or 2?: {unique_indices}")
        
        while True:
            action = cv2.waitKey(10)
            colour = cam.get_next_frame()
            # cv2.imshow(WIN_RF1, colour)
            
            
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

            # estimate pose
            est_pose = particle_filter.estimate_pose() # The estimate of the robots current pose
            # print(f"Estimated position: {est_pose}")
            
            # Draw map
            draw_world(est_pose, particle_filter, world)
            # Show world
            cv2.imshow(WIN_World, world)
            
            # If we are somewhat certain of where we are, then drive to given coordinate.
            if (particle_filter.evaluate_pose() < 5):
                vectorToDrive = (np.mean([landmarkIDS2[0][1], landmarkIDS2[1][1]]), np.mean([landmarkIDS2[0][2], landmarkIDS2[1][2]]))
                print(f"\n\nEstimated position[0],[1]: {est_pose[0], est_pose[1]}")
                print(f"\n\nVector to drive: {vectorToDrive}")
                Drive_dist = ((vectorToDrive[0] - est_pose[0])/100, (vectorToDrive[1] - est_pose[1])/100)
                print(f"\n\n Drive_dist: {Drive_dist} \n\n")
                roboarlo.RotateVector(Drive_dist-90)
                roboarlo.DriveVector(Drive_dist)
                
                return
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()

        # Clean-up capture thread
        cam.terminateCaptureThread()
        

if __name__ == '__main__':
    main()

    