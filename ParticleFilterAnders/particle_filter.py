import numpy as np

class ParticleFilter():
    def __init__(self, landmarks, cam, n = 1000):

        #Set cam
        self.cam = cam

        #Set landmarks dict

        self.landmarks = landmarks

        #Get bounds for the particles
        landmarks_array = np.array([[x,y] for (x,y) in landmarks.values()])

        self.min_x, self.max_x = np.min(landmarks_array[:,0]), np.max(landmarks_array[:,0])
        self.min_y, self.max_y = np.min(landmarks_array[:,1]), np.max(landmarks_array[:,1])


        self.particles = self.create_random_particles(n)

    def hardcode_init_pos(self):
        self.particles = np.array([150.0,0.0,np.pi(),0.0])

    def create_random_particles(self, n): 
        particles = np.empty([n, 4])
        particles[:,0] = np.random.uniform(self.min_x-50, self.max_x+50, n)
        particles[:,1] = np.random.uniform(self.min_y-50, self.max_y+50, n)
        particles[:,2] = np.mod(2.0*np.pi*np.random.rand(n), 2.0*np.pi)
        particles[:,3] = 1/n

        return particles


    def move_particles(self, delta_x, delta_y, delta_theta):
        self.particles[:,0] += delta_x
        self.particles[:,1] += delta_y
        self.particles[:,2] += delta_theta


    def add_uncertainty(self, sigma, sigma_theta): 
        self.particles[:,0] += np.random.randn(self.particles.shape[0])*sigma
        self.particles[:,1] += np.random.randn(self.particles.shape[0])*sigma
        self.particles[:,2] = np.mod(self.particles[:,2] + np.random.randn(self.particles.shape[0]) * sigma_theta, 2.0 * np.pi)


    def estimate_pose(self):

        pos = np.mean(self.particles,axis=0)[:2]

        sin_sum = np.sum(np.sin(self.particles[:,2]))
        cos_sum = np.sum(np.cos(self.particles[:,2]))
        angle = np.arctan2(sin_sum/self.particles.shape[0], cos_sum/self.particles.shape[0])

        return np.array([pos[0],pos[1],angle])


    def evaluate_pose(self):
        #The position part is probably overcomplicated
        mean_position = self.estimate_pose()[:2]
        distances = np.sqrt(np.sum((self.particles[:,:2] - mean_position)**2, axis=1))
        std_d = np.std(distances)

        std_theta = np.std(self.particles[:,2])

        return std_d + std_theta
    

    def reset_weights(self):
        self.particles[:,3] = 1/self.particles.shape[0]


    def MCL(self, objectIDs, dists, angles, self_localize = False):
        
        #Get all indices that is not a reoccurring objectID. 
        valid_indices = [i for i in range(len(objectIDs)) if 
            (i == 0 and objectIDs[i] in self.landmarks.keys()) or # First ID is always valid unless it is not in landmarks
            (objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in self.landmarks.keys())] # If ID is new it is valid unless it is not in landmarks
        # If there are no valid landmarks just return
        if (len(valid_indices) == 0):
            print("No valid landmarks")
            return
        
        #To get the product of weights, the initial weight must be 1
        self.particles[:,3] = 1.0

        # Compute particle weights for each detected landmark
        for i in valid_indices:
            curr_landmark, curr_dist, curr_angle = objectIDs[i], dists[i], angles[i]

            # Get sigma_d and sigma_theta
            sigma_d = 3 * np.std(np.sqrt(np.sum((self.landmarks[curr_landmark] - self.particles[:, :2])**2, axis=1)))
            sigma_theta = 2 * np.std(self.particles[:, 2])

            # Distance-part of weight
            measured_dists = np.sqrt(np.sum((self.landmarks[curr_landmark] - self.particles[:, :2])**2, axis=1))
            norm_const_d = 1 / (np.sqrt(2 * np.pi * (sigma_d**2)))
            exp_term_d = np.exp(-((measured_dists - curr_dist)**2) / (2 * (sigma_d**2)))
            weights_d = norm_const_d * exp_term_d

            # Angle-part of weight
            e_i_theta = np.column_stack([np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])])
            e_i_theta_hat = np.column_stack([-np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2])])
            e_i_l = (self.landmarks[curr_landmark] - self.particles[:, :2]) / curr_dist
            theta_i = np.sign(np.sum(e_i_l * e_i_theta_hat, axis=1)) * np.arccos(np.sum(e_i_l * e_i_theta, axis=1))
            theta_i[np.isnan(theta_i)] = 0.0
            norm_const_theta = 1 / (np.sqrt(2 * np.pi * (sigma_theta**2)))
            exp_term_theta = np.exp(-((curr_angle - theta_i)**2) / (2 * (sigma_theta**2)))
            weights_theta = norm_const_theta * exp_term_theta

            #Update particle weight
            epsilon = 0.00000001 # Hashtag numerical stability
            weights = weights_d * weights_theta + epsilon
            self.particles[:, 3] *= weights  # Weight is multiplied onto existing weight, as we consider multiple landmarks

            #"Histogramize" the top 10%. Reasonning: We want the top x% to be evenly weighted in the probability distribution, in order to slow down the convergence on a single point. This creates the circle when looking at only one point
            # If we are -very unsure- (very large variance), then we must remain open to multiple good options. When sufficently low, we can skip this and focus onto a point.
        if (self_localize): 
            sorted_indices = np.argsort(self.particles[:, 3]) # Get the indices to sort by weight            
            self.particles = self.particles[sorted_indices] # sort
            rows_to_change = int(len(self.particles) * 0.1) #10%
            self.particles[-rows_to_change:, 3] = np.mean(self.particles[-rows_to_change:, 3])

        # Define a normalized probability distribution
        probabilities = self.particles[:,3] / np.sum(self.particles[:,3])

        # Now draw new particles:
        selected_indices = np.random.choice(self.particles.shape[0], size=self.particles.shape[0], p=probabilities)

        # Update the particles array with the resampled particles
        self.particles = self.particles[selected_indices]


    def perform_MCL(self, n, self_localize = False):

        if (self_localize):
            #increase number of particles temporarily
            pass

        for _ in range(n):

            # Fetch next frame
            colour = self.cam.get_next_frame()
            
            # Detect objects
            objectIDs, dists, angles = self.cam.detect_aruco_objects(colour)

            if not isinstance(objectIDs, type(None)): # if there is actually work to do..

                self.MCL(objectIDs, dists, angles, self_localize)

                self.add_uncertainty(0.5,0.1)

            else:
                # No observation - reset weights to uniform distribution
                self.reset_weights()

                self.add_uncertainty(0.1,0.1)

                #maybe return here, deends on the definition of the MCL algoritm
