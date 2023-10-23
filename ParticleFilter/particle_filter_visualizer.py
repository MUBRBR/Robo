import cv2
import camera
import numpy as np

import particle_filter as pf

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

    landmark_colors = [CRED, CGREEN]

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
    for key, (l_x, l_y) in particle_filter.landmarks.items():
        lm = (int(l_x + offsetX), int(ymax - (l_y + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[key - 1], 2)

    # Draw estimated robot pose
    a = (int(est_pose[0]) + offsetX, ymax - (int(est_pose[1]) + offsetY))
    b = (int(est_pose[0] + 15.0 * np.cos(est_pose[2])) + offsetX, ymax - (int(est_pose[1] + 15.0 * np.sin(est_pose[2])) + offsetY))

    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)

# Main program #
try:
    # Open windows
    WIN_RF1 = "Robot view"
    cv2.namedWindow(WIN_RF1)
    cv2.moveWindow(WIN_RF1, 50, 50)

    WIN_World = "World view"
    cv2.namedWindow(WIN_World)
    cv2.moveWindow(WIN_World, 500, 50)

    # Initialize particles
    num_particles = 10000
    landmarks = [[0.0,0.0],[300.0,0.0]]
    particle_filter = pf.ParticleFilter([0,0],[1,1],landmarks,num_particles)

    est_pose = particle_filter.estimate_pose() # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particle_filter, world)

    cam = camera.Camera(0, 'arlo', useCaptureThread = True)

    while True:

        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
    
        if True: # cause we aint runnin' on arlo bois
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2

        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)

        print(f"Meassure of how sure we are of the current estimated pose: {particle_filter.evaluate_pose()}")

        if not isinstance(objectIDs, type(None)): # if there is actually work to do..

            particle_filter.MCL(objectIDs, dists, angles)

            particle_filter.add_uncertainty(0.5,0.1)

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            particle_filter.reset_weights()

            particle_filter.add_uncertainty(1,0.1)

        est_pose = particle_filter.estimate_pose() # The estimate of the robots current pose

        # Either way draw map
        draw_world(est_pose, particle_filter, world)

        # Show frame
        cv2.imshow(WIN_RF1, colour)

        # Show world
        cv2.imshow(WIN_World, world)

  
finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

